// Original code from jizhang-cmu/autonomy_stack_mecanum_wheel_platform.
// Modified by Lihan Chen on 2026/01/06
// Copyright 2025 Ji Zhang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_NODE_HPP_
#define TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "example_interfaces/msg/float32.hpp"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace terrain_analysis
{

struct VehiclePose
{
  double x, y, z;
  double roll, pitch, yaw;
  double sin_roll, cos_roll;
  double sin_pitch, cos_pitch;
  double sin_yaw, cos_yaw;

  explicit VehiclePose(const Eigen::Affine3d & pose)
  {
    x = pose.translation().x();
    y = pose.translation().y();
    z = pose.translation().z();

    Eigen::Vector3d euler = pose.rotation().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
    roll = euler[2];

    sin_roll = std::sin(roll);
    cos_roll = std::cos(roll);
    sin_pitch = std::sin(pitch);
    cos_pitch = std::cos(pitch);
    sin_yaw = std::sin(yaw);
    cos_yaw = std::cos(yaw);
  }
};

struct VoxelIndex
{
  int x, y;

  inline bool isValid(int width) const { return x >= 0 && x < width && y >= 0 && y < width; }
  inline int toLinear(int width) const { return width * x + y; }
};

class TerrainAnalysisNode : public rclcpp::Node
{
public:
  explicit TerrainAnalysisNode(const rclcpp::NodeOptions & options);

private:
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud_msg);
  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
  void clearingHandler(const example_interfaces::msg::Float32::ConstSharedPtr dis);

  void processTerrainAnalysis(const VehiclePose & pose);
  bool getLidarPose(const rclcpp::Time & stamp, Eigen::Affine3d & lidar_pose_in_odom);

  void rollTerrainVoxels(const VehiclePose & pose);
  void stackLaserScans(const VehiclePose & pose);
  void downsampleAndDecayVoxels(const VehiclePose & pose, bool clearing);
  void estimateGroundElevation(const VehiclePose & pose);
  void detectDynamicObstacles(const VehiclePose & pose);
  void buildTerrainWithElevation(const VehiclePose & pose);
  void generateNoDataObstacles(const VehiclePose & pose);

  inline VoxelIndex computeTerrainIndex(double px, double py, const VehiclePose & pose) const
  {
    return {
      static_cast<int>((px - pose.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
        K_TERRAIN_VOXEL_HALF_WIDTH,
      static_cast<int>((py - pose.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
        K_TERRAIN_VOXEL_HALF_WIDTH};
  }

  inline VoxelIndex computePlanarIndex(double px, double py, const VehiclePose & pose) const
  {
    return {
      static_cast<int>((px - pose.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        K_PLANAR_VOXEL_HALF_WIDTH,
      static_cast<int>((py - pose.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        K_PLANAR_VOXEL_HALF_WIDTH};
  }

  // ROS 2 publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joystick_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr sub_clearing_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  double scan_voxel_size_;
  double decay_time_;
  double no_decay_dis_;
  double clearing_dis_;
  bool use_sorting_;
  double quantile_z_;
  bool consider_drop_;
  bool limit_ground_lift_;
  double max_ground_lift_;
  bool clear_dy_obs_;
  double min_dy_obs_dis_;
  double abs_dy_obs_rel_z_thre_;
  double min_dy_obs_vfov_;
  double max_dy_obs_vfov_;
  int min_dy_obs_point_num_;
  int min_out_of_fov_point_num_;
  double obstacle_height_thre_;
  bool no_data_obstacle_;
  int no_data_block_skip_num_;
  int min_block_point_num_;
  double vehicle_height_;
  int voxel_point_update_thre_;
  double voxel_time_update_thre_;
  double min_rel_z_;
  double max_rel_z_;
  double dis_ratio_z_;

  std::string odom_frame_;
  std::string lidar_frame_;

  // Terrain voxel parameters
  float terrain_voxel_size_;
  int terrain_voxel_shift_x_;
  int terrain_voxel_shift_y_;
  static constexpr int K_TERRAIN_VOXEL_WIDTH = 21;
  static constexpr int K_TERRAIN_VOXEL_HALF_WIDTH = (K_TERRAIN_VOXEL_WIDTH - 1) / 2;
  static constexpr int K_TERRAIN_VOXEL_NUM = K_TERRAIN_VOXEL_WIDTH * K_TERRAIN_VOXEL_WIDTH;

  // Planar voxel parameters
  float planar_voxel_size_;
  static constexpr int K_PLANAR_VOXEL_WIDTH = 51;
  static constexpr int K_PLANAR_VOXEL_HALF_WIDTH = (K_PLANAR_VOXEL_WIDTH - 1) / 2;
  static constexpr int K_PLANAR_VOXEL_NUM = K_PLANAR_VOXEL_WIDTH * K_PLANAR_VOXEL_WIDTH;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_crop_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_dwz_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> terrain_voxel_cloud_;

  // Voxel data
  std::vector<int> terrain_voxel_update_num_;
  std::vector<float> terrain_voxel_update_time_;
  std::vector<float> planar_voxel_elev_;
  std::vector<int> planar_voxel_edge_;
  std::vector<int> planar_voxel_dy_obs_;
  std::vector<int> planar_voxel_out_of_fov_;
  std::vector<std::vector<float>> planar_point_elev_;

  // State variables
  double laser_cloud_time_;
  bool new_laser_cloud_;
  double system_init_time_;
  bool system_inited_;
  int no_data_inited_;
  Eigen::Affine3d vehicle_pose_rec_;

  // PCL filter
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;
};

}  // namespace terrain_analysis

#endif  // TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_NODE_HPP_
