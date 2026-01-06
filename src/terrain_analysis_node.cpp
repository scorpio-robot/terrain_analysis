// Original code from jizhang-cmu/autonomy_stack_mecanum_wheel_platform.
// Modified by Lihan Chen on 2026/01/06
// Copyright 2025 Ji Zhang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include "terrain_analysis/terrain_analysis_node.hpp"

#include <algorithm>
#include <cmath>

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace terrain_analysis
{

TerrainAnalysisNode::TerrainAnalysisNode(const rclcpp::NodeOptions & options)
: Node("terrain_analysis", options),
  terrain_voxel_size_(1.0),
  terrain_voxel_shift_x_(0),
  terrain_voxel_shift_y_(0),
  planar_voxel_size_(0.2),
  laser_cloud_time_(0.0),
  new_laser_cloud_(false),
  system_init_time_(0.0),
  system_inited_(false),
  no_data_inited_(0)
{
  this->declare_parameter<double>("scan_voxel_size", 0.05);
  this->declare_parameter<double>("decay_time", 2.0);
  this->declare_parameter<double>("no_decay_dis", 4.0);
  this->declare_parameter<double>("clearing_dis", 8.0);
  this->declare_parameter<bool>("use_sorting", true);
  this->declare_parameter<double>("quantile_z", 0.25);
  this->declare_parameter<bool>("consider_drop", false);
  this->declare_parameter<bool>("limit_ground_lift", false);
  this->declare_parameter<double>("max_ground_lift", 0.15);
  this->declare_parameter<bool>("clear_dy_obs", false);
  this->declare_parameter<double>("min_dy_obs_dis", 0.3);
  this->declare_parameter<double>("abs_dy_obs_rel_z_thre", 0.2);
  this->declare_parameter<double>("min_dy_obs_vfov", -16.0);
  this->declare_parameter<double>("max_dy_obs_vfov", 16.0);
  this->declare_parameter<int>("min_dy_obs_point_num", 1);
  this->declare_parameter<int>("min_out_of_fov_point_num", 2);
  this->declare_parameter<double>("obstacle_height_thre", 0.2);
  this->declare_parameter<bool>("no_data_obstacle", false);
  this->declare_parameter<int>("no_data_block_skip_num", 0);
  this->declare_parameter<int>("min_block_point_num", 10);
  this->declare_parameter<double>("vehicle_height", 1.5);
  this->declare_parameter<int>("voxel_point_update_thre", 100);
  this->declare_parameter<double>("voxel_time_update_thre", 2.0);
  this->declare_parameter<double>("min_rel_z", -1.5);
  this->declare_parameter<double>("max_rel_z", 0.2);
  this->declare_parameter<double>("dis_ratio_z", 0.2);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("lidar_frame", "mid360");

  this->get_parameter("scan_voxel_size", scan_voxel_size_);
  this->get_parameter("decay_time", decay_time_);
  this->get_parameter("no_decay_dis", no_decay_dis_);
  this->get_parameter("clearing_dis", clearing_dis_);
  this->get_parameter("use_sorting", use_sorting_);
  this->get_parameter("quantile_z", quantile_z_);
  this->get_parameter("consider_drop", consider_drop_);
  this->get_parameter("limit_ground_lift", limit_ground_lift_);
  this->get_parameter("max_ground_lift", max_ground_lift_);
  this->get_parameter("clear_dy_obs", clear_dy_obs_);
  this->get_parameter("min_dy_obs_dis", min_dy_obs_dis_);
  this->get_parameter("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
  this->get_parameter("min_dy_obs_vfov", min_dy_obs_vfov_);
  this->get_parameter("max_dy_obs_vfov", max_dy_obs_vfov_);
  this->get_parameter("min_dy_obs_point_num", min_dy_obs_point_num_);
  this->get_parameter("min_out_of_fov_point_num", min_out_of_fov_point_num_);
  this->get_parameter("obstacle_height_thre", obstacle_height_thre_);
  this->get_parameter("no_data_obstacle", no_data_obstacle_);
  this->get_parameter("no_data_block_skip_num", no_data_block_skip_num_);
  this->get_parameter("min_block_point_num", min_block_point_num_);
  this->get_parameter("vehicle_height", vehicle_height_);
  this->get_parameter("voxel_point_update_thre", voxel_point_update_thre_);
  this->get_parameter("voxel_time_update_thre", voxel_time_update_thre_);
  this->get_parameter("min_rel_z", min_rel_z_);
  this->get_parameter("max_rel_z", max_rel_z_);
  this->get_parameter("dis_ratio_z", dis_ratio_z_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);

  // Initialize point clouds
  laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_crop_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_dwz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  // Initialize terrain voxel clouds
  terrain_voxel_cloud_.resize(K_TERRAIN_VOXEL_NUM);
  for (int i = 0; i < K_TERRAIN_VOXEL_NUM; ++i) {
    terrain_voxel_cloud_[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  // Initialize voxel data
  terrain_voxel_update_num_.resize(K_TERRAIN_VOXEL_NUM, 0);
  terrain_voxel_update_time_.resize(K_TERRAIN_VOXEL_NUM, 0.0f);
  planar_voxel_elev_.resize(K_PLANAR_VOXEL_NUM, 0.0f);
  planar_voxel_edge_.resize(K_PLANAR_VOXEL_NUM, 0);
  planar_voxel_dy_obs_.resize(K_PLANAR_VOXEL_NUM, 0);
  planar_voxel_out_of_fov_.resize(K_PLANAR_VOXEL_NUM, 0);
  planar_point_elev_.resize(K_PLANAR_VOXEL_NUM);

  // Configure downsampling filter
  down_size_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscribers
  sub_laser_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 5,
    std::bind(&TerrainAnalysisNode::laserCloudHandler, this, std::placeholders::_1));

  sub_joystick_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 5, std::bind(&TerrainAnalysisNode::joystickHandler, this, std::placeholders::_1));

  sub_clearing_ = this->create_subscription<example_interfaces::msg::Float32>(
    "map_clearing", 5,
    std::bind(&TerrainAnalysisNode::clearingHandler, this, std::placeholders::_1));

  // Create publisher
  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map", 2);

  RCLCPP_INFO(this->get_logger(), "Terrain Analysis Node initialized");
}

bool TerrainAnalysisNode::getLidarPose(
  const rclcpp::Time & stamp, Eigen::Affine3d & lidar_pose_in_odom)
{
  try {
    // Get transform from odom to lidar
    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
      odom_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
    lidar_pose_in_odom = tf2::transformToEigen(transform_stamped.transform);

    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "TF lookup failed: %s", ex.what());
    return false;
  }
}

void TerrainAnalysisNode::joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons.size() > 5 && joy->buttons[5] > 0.5) {
    no_data_inited_ = 0;
    new_laser_cloud_ = true;  // Trigger clearing in next processing cycle
    RCLCPP_INFO(this->get_logger(), "Clearing cloud triggered by joystick");
  }
}

void TerrainAnalysisNode::clearingHandler(
  const example_interfaces::msg::Float32::ConstSharedPtr dis)
{
  no_data_inited_ = 0;
  clearing_dis_ = dis->data;
  new_laser_cloud_ = true;  // Trigger clearing in next processing cycle
  RCLCPP_INFO(this->get_logger(), "Clearing cloud with distance: %.2f", clearing_dis_);
}

void TerrainAnalysisNode::laserCloudHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud_msg)
{
  laser_cloud_time_ = rclcpp::Time(laser_cloud_msg->header.stamp).seconds();
  if (!system_inited_) {
    system_init_time_ = laser_cloud_time_;
    system_inited_ = true;
  }

  Eigen::Affine3d lidar_pose_in_odom;
  if (!getLidarPose(laser_cloud_msg->header.stamp, lidar_pose_in_odom)) {
    return;
  }

  const VehiclePose pose(lidar_pose_in_odom);

  // Track vehicle movement for no-data obstacle detection
  if (no_data_inited_ == 0) {
    vehicle_pose_rec_ = lidar_pose_in_odom;
    no_data_inited_ = 1;
  } else if (no_data_inited_ == 1) {
    const double dis =
      (lidar_pose_in_odom.translation() - vehicle_pose_rec_.translation()).head<2>().norm();
    if (dis >= no_decay_dis_) no_data_inited_ = 2;
  }

  pcl::fromROSMsg(*laser_cloud_msg, *laser_cloud_);

  laser_cloud_crop_->clear();
  for (const auto & point : laser_cloud_->points) {
    const double dis = std::hypot(point.x - pose.x, point.y - pose.y);
    const double rel_z = point.z - pose.z;

    if (
      rel_z > min_rel_z_ - dis_ratio_z_ * dis && rel_z < max_rel_z_ + dis_ratio_z_ * dis &&
      dis < terrain_voxel_size_ * (K_TERRAIN_VOXEL_HALF_WIDTH + 1)) {
      pcl::PointXYZI new_point;
      new_point.x = point.x;
      new_point.y = point.y;
      new_point.z = point.z;
      new_point.intensity = laser_cloud_time_ - system_init_time_;
      laser_cloud_crop_->push_back(new_point);
    }
  }

  new_laser_cloud_ = true;
  processTerrainAnalysis(pose);
}

void TerrainAnalysisNode::processTerrainAnalysis(const VehiclePose & pose)
{
  if (!new_laser_cloud_) return;
  new_laser_cloud_ = false;

  const bool clearing = (no_data_inited_ == 0);

  rollTerrainVoxels(pose);
  stackLaserScans(pose);
  downsampleAndDecayVoxels(pose, clearing);

  // Collect terrain cloud from nearby voxels
  terrain_cloud_->clear();
  for (int ind_x = K_TERRAIN_VOXEL_HALF_WIDTH - 5; ind_x <= K_TERRAIN_VOXEL_HALF_WIDTH + 5;
       ++ind_x) {
    for (int ind_y = K_TERRAIN_VOXEL_HALF_WIDTH - 5; ind_y <= K_TERRAIN_VOXEL_HALF_WIDTH + 5;
         ++ind_y) {
      *terrain_cloud_ += *terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * ind_x + ind_y];
    }
  }

  estimateGroundElevation(pose);
  if (clear_dy_obs_) detectDynamicObstacles(pose);
  buildTerrainWithElevation(pose);
  if (no_data_obstacle_ && no_data_inited_ == 2) generateNoDataObstacles(pose);

  // Publish
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*terrain_cloud_elev_, msg);
  msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
  msg.header.frame_id = odom_frame_;
  pub_laser_cloud_->publish(msg);
}

void TerrainAnalysisNode::rollTerrainVoxels(const VehiclePose & pose)
{
  // Roll X negative
  while (pose.x - terrain_voxel_size_ * terrain_voxel_shift_x_ < -terrain_voxel_size_) {
    for (int y = 0; y < K_TERRAIN_VOXEL_WIDTH; ++y) {
      auto temp = terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * (K_TERRAIN_VOXEL_WIDTH - 1) + y];
      for (int x = K_TERRAIN_VOXEL_WIDTH - 1; x >= 1; --x) {
        terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + y] =
          terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * (x - 1) + y];
      }
      terrain_voxel_cloud_[y] = temp;
      temp->clear();
    }
    terrain_voxel_shift_x_--;
  }

  // Roll X positive
  while (pose.x - terrain_voxel_size_ * terrain_voxel_shift_x_ > terrain_voxel_size_) {
    for (int y = 0; y < K_TERRAIN_VOXEL_WIDTH; ++y) {
      auto temp = terrain_voxel_cloud_[y];
      for (int x = 0; x < K_TERRAIN_VOXEL_WIDTH - 1; ++x) {
        terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + y] =
          terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * (x + 1) + y];
      }
      terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * (K_TERRAIN_VOXEL_WIDTH - 1) + y] = temp;
      temp->clear();
    }
    terrain_voxel_shift_x_++;
  }

  // Roll Y negative
  while (pose.y - terrain_voxel_size_ * terrain_voxel_shift_y_ < -terrain_voxel_size_) {
    for (int x = 0; x < K_TERRAIN_VOXEL_WIDTH; ++x) {
      auto temp = terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + (K_TERRAIN_VOXEL_WIDTH - 1)];
      for (int y = K_TERRAIN_VOXEL_WIDTH - 1; y >= 1; --y) {
        terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + y] =
          terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + (y - 1)];
      }
      terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x] = temp;
      temp->clear();
    }
    terrain_voxel_shift_y_--;
  }

  // Roll Y positive
  while (pose.y - terrain_voxel_size_ * terrain_voxel_shift_y_ > terrain_voxel_size_) {
    for (int x = 0; x < K_TERRAIN_VOXEL_WIDTH; ++x) {
      auto temp = terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x];
      for (int y = 0; y < K_TERRAIN_VOXEL_WIDTH - 1; ++y) {
        terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + y] =
          terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + (y + 1)];
      }
      terrain_voxel_cloud_[K_TERRAIN_VOXEL_WIDTH * x + (K_TERRAIN_VOXEL_WIDTH - 1)] = temp;
      temp->clear();
    }
    terrain_voxel_shift_y_++;
  }
}

void TerrainAnalysisNode::stackLaserScans(const VehiclePose & pose)
{
  for (const auto & point : laser_cloud_crop_->points) {
    const auto idx = computeTerrainIndex(point.x, point.y, pose);
    if (idx.isValid(K_TERRAIN_VOXEL_WIDTH)) {
      const int linear = idx.toLinear(K_TERRAIN_VOXEL_WIDTH);
      terrain_voxel_cloud_[linear]->push_back(point);
      terrain_voxel_update_num_[linear]++;
    }
  }
}

void TerrainAnalysisNode::downsampleAndDecayVoxels(const VehiclePose & pose, bool clearing)
{
  for (int ind = 0; ind < K_TERRAIN_VOXEL_NUM; ++ind) {
    if (
      terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
      laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >=
        voxel_time_update_thre_ ||
      clearing) {
      auto & voxel = terrain_voxel_cloud_[ind];

      laser_cloud_dwz_->clear();
      down_size_filter_.setInputCloud(voxel);
      down_size_filter_.filter(*laser_cloud_dwz_);

      voxel->clear();
      for (const auto & point : laser_cloud_dwz_->points) {
        const double dis = std::hypot(point.x - pose.x, point.y - pose.y);
        const double rel_z = point.z - pose.z;

        const bool in_height_range =
          rel_z > min_rel_z_ - dis_ratio_z_ * dis && rel_z < max_rel_z_ + dis_ratio_z_ * dis;
        const bool not_expired =
          laser_cloud_time_ - system_init_time_ - point.intensity < decay_time_ ||
          dis < no_decay_dis_;
        const bool not_clearing = dis >= clearing_dis_ || !clearing;

        if (in_height_range && not_expired && not_clearing) {
          voxel->push_back(point);
        }
      }

      terrain_voxel_update_num_[ind] = 0;
      terrain_voxel_update_time_[ind] = laser_cloud_time_ - system_init_time_;
    }
  }
}

void TerrainAnalysisNode::estimateGroundElevation(const VehiclePose & pose)
{
  // Reset voxel data
  for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
    planar_voxel_elev_[i] = 0.0f;
    planar_voxel_edge_[i] = 0;
    planar_voxel_dy_obs_[i] = 0;
    planar_voxel_out_of_fov_[i] = 0;
    planar_point_elev_[i].clear();
  }

  // Collect elevation samples
  for (const auto & point : terrain_cloud_->points) {
    if (point.z - pose.z < min_rel_z_ || point.z - pose.z > max_rel_z_) continue;

    const auto idx = computePlanarIndex(point.x, point.y, pose);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const VoxelIndex neighbor = {idx.x + dx, idx.y + dy};
        if (neighbor.isValid(K_PLANAR_VOXEL_WIDTH)) {
          planar_point_elev_[neighbor.toLinear(K_PLANAR_VOXEL_WIDTH)].push_back(point.z);
        }
      }
    }
  }

  if (use_sorting_) {
    for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
      auto & elevs = planar_point_elev_[i];
      if (elevs.empty()) continue;

      std::sort(elevs.begin(), elevs.end());
      const int quantile_id =
        std::min(static_cast<int>(quantile_z_ * elevs.size()), static_cast<int>(elevs.size()) - 1);

      planar_voxel_elev_[i] = elevs[quantile_id];
      if (limit_ground_lift_ && planar_voxel_elev_[i] > elevs[0] + max_ground_lift_) {
        planar_voxel_elev_[i] = elevs[0] + max_ground_lift_;
      }
    }
  } else {
    for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
      const auto & elevs = planar_point_elev_[i];
      if (!elevs.empty()) {
        planar_voxel_elev_[i] = *std::min_element(elevs.begin(), elevs.end());
      }
    }
  }
}

void TerrainAnalysisNode::detectDynamicObstacles(const VehiclePose & pose)
{
  // Check terrain cloud for dynamic obstacles
  for (const auto & point : terrain_cloud_->points) {
    const auto idx = computePlanarIndex(point.x, point.y, pose);
    if (!idx.isValid(K_PLANAR_VOXEL_WIDTH)) continue;

    const int linear = idx.toLinear(K_PLANAR_VOXEL_WIDTH);
    const float point_x = point.x - pose.x;
    const float point_y = point.y - pose.y;
    const float point_z = point.z - pose.z;
    const float dis = std::hypot(point_x, point_y);

    if (dis <= min_dy_obs_dis_) {
      planar_voxel_dy_obs_[linear] += min_dy_obs_point_num_;
      continue;
    }

    const float h = point.z - planar_voxel_elev_[linear];
    if (h <= obstacle_height_thre_) continue;

    // Transform to vehicle frame for FOV check
    const float x2 = point_x * pose.cos_yaw + point_y * pose.sin_yaw;
    const float y2 = -point_x * pose.sin_yaw + point_y * pose.cos_yaw;
    const float x3 = x2 * pose.cos_pitch - point_z * pose.sin_pitch;
    const float y3 = y2;
    const float z3 = x2 * pose.sin_pitch + point_z * pose.cos_pitch;
    const float y4 = y3 * pose.cos_roll + z3 * pose.sin_roll;
    const float z4 = -y3 * pose.sin_roll + z3 * pose.cos_roll;

    const float angle_deg = std::atan2(z4, std::hypot(x3, y4)) * 180.0 / M_PI;

    if (
      (angle_deg > min_dy_obs_vfov_ && angle_deg < max_dy_obs_vfov_) ||
      std::fabs(z4) < abs_dy_obs_rel_z_thre_) {
      planar_voxel_dy_obs_[linear]++;
    } else if (angle_deg <= min_dy_obs_vfov_) {
      planar_voxel_out_of_fov_[linear]++;
    }
  }

  // Mark recent scan points as non-obstacles
  for (const auto & point : laser_cloud_crop_->points) {
    const auto idx = computePlanarIndex(point.x, point.y, pose);
    if (!idx.isValid(K_PLANAR_VOXEL_WIDTH)) continue;

    const int linear = idx.toLinear(K_PLANAR_VOXEL_WIDTH);
    const float h = point.z - planar_voxel_elev_[linear];
    if (h > obstacle_height_thre_) {
      planar_voxel_dy_obs_[linear] = -1;
    }
  }
}

void TerrainAnalysisNode::buildTerrainWithElevation(const VehiclePose & pose)
{
  terrain_cloud_elev_->clear();

  for (auto point : terrain_cloud_->points) {
    const double rel_z = point.z - pose.z;
    if (rel_z <= min_rel_z_ || rel_z >= max_rel_z_) continue;

    const auto idx = computePlanarIndex(point.x, point.y, pose);
    if (!idx.isValid(K_PLANAR_VOXEL_WIDTH)) continue;

    const int linear = idx.toLinear(K_PLANAR_VOXEL_WIDTH);
    const int dy_obs_count = planar_voxel_dy_obs_[linear];
    if (clear_dy_obs_ && dy_obs_count >= min_dy_obs_point_num_) continue;

    float elev_diff = point.z - planar_voxel_elev_[linear];
    if (consider_drop_) elev_diff = std::fabs(elev_diff);

    const int point_count = planar_point_elev_[linear].size();
    const int out_of_fov_count = planar_voxel_out_of_fov_[linear];

    const bool has_sufficient_data = point_count >= min_block_point_num_;
    const bool within_vehicle = elev_diff >= 0 && elev_diff < vehicle_height_;
    const bool not_blocked = !clear_dy_obs_ || dy_obs_count < 0 ||
                             elev_diff < obstacle_height_thre_ ||
                             out_of_fov_count >= min_out_of_fov_point_num_;

    if (has_sufficient_data && within_vehicle && not_blocked) {
      point.intensity = elev_diff;
      terrain_cloud_elev_->push_back(point);
    }
  }
}

void TerrainAnalysisNode::generateNoDataObstacles(const VehiclePose & pose)
{
  // Mark voxels with insufficient data
  for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
    if (planar_point_elev_[i].size() < static_cast<size_t>(min_block_point_num_)) {
      planar_voxel_edge_[i] = 1;
    }
  }

  // Expand edge markers
  for (int skip = 0; skip < no_data_block_skip_num_; ++skip) {
    for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
      if (planar_voxel_edge_[i] < 1) continue;

      const int x = i / K_PLANAR_VOXEL_WIDTH;
      const int y = i % K_PLANAR_VOXEL_WIDTH;
      bool is_edge = false;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const VoxelIndex neighbor = {x + dx, y + dy};
          if (neighbor.isValid(K_PLANAR_VOXEL_WIDTH)) {
            if (
              planar_voxel_edge_[neighbor.toLinear(K_PLANAR_VOXEL_WIDTH)] < planar_voxel_edge_[i]) {
              is_edge = true;
            }
          }
        }
      }

      if (!is_edge) planar_voxel_edge_[i]++;
    }
  }

  // Generate obstacle points for no-data regions
  for (int i = 0; i < K_PLANAR_VOXEL_NUM; ++i) {
    if (planar_voxel_edge_[i] <= no_data_block_skip_num_) continue;

    const int x = i / K_PLANAR_VOXEL_WIDTH;
    const int y = i % K_PLANAR_VOXEL_WIDTH;

    pcl::PointXYZI point;
    point.x =
      planar_voxel_size_ * (x - K_PLANAR_VOXEL_HALF_WIDTH) + pose.x - planar_voxel_size_ / 4.0;
    point.y =
      planar_voxel_size_ * (y - K_PLANAR_VOXEL_HALF_WIDTH) + pose.y - planar_voxel_size_ / 4.0;
    point.z = pose.z;
    point.intensity = vehicle_height_;

    terrain_cloud_elev_->push_back(point);
    point.x += planar_voxel_size_ / 2.0;
    terrain_cloud_elev_->push_back(point);
    point.y += planar_voxel_size_ / 2.0;
    terrain_cloud_elev_->push_back(point);
    point.x -= planar_voxel_size_ / 2.0;
    terrain_cloud_elev_->push_back(point);
  }
}

}  // namespace terrain_analysis

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisNode)
