# Terrain Analysis

[![License](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A ROS2 package for real-time terrain analysis using LiDAR point clouds, providing elevation maps for autonomous navigation in complex environments.

## Overview

The `terrain_analysis` package processes registered LiDAR point clouds to generate detailed terrain elevation maps. It analyzes point cloud data to estimate ground elevation, detect dynamic obstacles, and identify traversable terrain regions. The package is designed for real-time operation in autonomous navigation systems, supporting both indoor and outdoor environments.

## Features

- **Real-time Terrain Mapping**: Processes LiDAR scans to create elevation-based terrain maps
- **Ground Elevation Estimation**: Uses quantile-based or minimum elevation estimation for accurate ground plane detection
- **Dynamic Obstacle Detection**: Optional detection and filtering of moving obstacles based on height and field-of-view criteria
- **Point Cloud Decay**: Automatically removes outdated measurements over time to maintain map freshness
- **No-Data Obstacle Generation**: Creates virtual obstacles in regions with insufficient sensor data
- **TF2 Integration**: Uses modern ROS2 transform system for accurate pose estimation
- **Configurable Voxel Processing**: Adjustable voxel sizes and processing parameters for different environments

## Inputs

### Topics Subscribed

- **`registered_scan`** (`sensor_msgs/PointCloud2`)
  - Registered LiDAR point cloud data
  - Expected frame: LiDAR sensor frame (automatically detected from message header)
  - Used for real-time terrain analysis and mapping

- **`joy`** (`sensor_msgs/Joy`)
  - Joystick input for manual triggering of map clearing
  - Button 5 (index 5) triggers clearing when pressed

- **`map_clearing`** (`example_interfaces/Float32`)
  - Distance threshold for map clearing operations
  - Triggers clearing of terrain data beyond the specified distance

### TF Requirements

- **`odom` → `lidar_frame`**: Transform from odometry frame to LiDAR sensor frame
  - Required for accurate pose estimation during terrain analysis
  - `lidar_frame` defaults to `mid360` but can be configured

## Outputs

### Topics Published

- **`terrain_map`** (`sensor_msgs/PointCloud2`)
  - Terrain elevation map
  - Frame: `odom`
  - Point intensity represents elevation difference from estimated ground plane
  - Includes traversable terrain points and obstacle markers

## Parameters

### Frame Configuration

- **`odom_frame`** (string, default: "odom")
  - Name of the odometry coordinate frame

- **`lidar_frame`** (string, default: "mid360")
  - Name of the LiDAR sensor frame

### Voxel Processing Parameters

- **`scan_voxel_size`** (double, default: 0.05)
  - Size of scan downsampling voxels in meters

- **`terrain_voxel_size`** (double, default: 1.0)
  - Size of terrain voxels in meters (fixed internally)

- **`planar_voxel_size`** (double, default: 0.2)
  - Size of planar analysis voxels in meters (fixed internally)

### Decay and Clearing Parameters

- **`decay_time`** (double, default: 2.0)
  - Time threshold for point decay in seconds

- **`no_decay_dis`** (double, default: 4.0)
  - Distance within which points do not decay

- **`clearing_dis`** (double, default: 8.0)
  - Distance threshold for clearing operations

### Ground Elevation Parameters

- **`use_sorting`** (bool, default: true)
  - Enable quantile-based ground elevation estimation

- **`quantile_z`** (double, default: 0.25)
  - Quantile value for elevation estimation (0.0-1.0)

- **`consider_drop`** (bool, default: false)
  - Consider negative elevation differences as obstacles

- **`limit_ground_lift`** (bool, default: false)
  - Limit maximum ground elevation lift

- **`max_ground_lift`** (double, default: 0.15)
  - Maximum allowed ground lift in meters

### Dynamic Obstacle Detection Parameters

- **`clear_dy_obs`** (bool, default: false)
  - Enable dynamic obstacle detection and filtering

- **`min_dy_obs_dis`** (double, default: 0.3)
  - Minimum distance for dynamic obstacle detection

- **`abs_dy_obs_rel_z_thre`** (double, default: 0.2)
  - Absolute relative Z threshold for dynamic obstacles

- **`min_dy_obs_vfov`** (double, default: -16.0)
  - Minimum vertical field-of-view for obstacle detection (degrees)

- **`max_dy_obs_vfov`** (double, default: 16.0)
  - Maximum vertical field-of-view for obstacle detection (degrees)

- **`min_dy_obs_point_num`** (int, default: 1)
  - Minimum number of points to classify as dynamic obstacle

- **`min_out_of_fov_point_num`** (int, default: 2)
  - Minimum number of out-of-FOV points to allow obstacle

- **`obstacle_height_thre`** (double, default: 0.2)
  - Height threshold for obstacle classification

### No-Data Obstacle Parameters

- **`no_data_obstacle`** (bool, default: false)
  - Enable generation of obstacles in no-data regions

- **`no_data_block_skip_num`** (int, default: 0)
  - Number of expansion iterations for no-data regions

- **`min_block_point_num`** (int, default: 10)
  - Minimum points required per voxel block

### Vehicle and Geometric Parameters

- **`vehicle_height`** (double, default: 1.5)
  - Height of the vehicle for traversability checks

- **`voxel_point_update_thre`** (int, default: 100)
  - Point count threshold for voxel updates

- **`voxel_time_update_thre`** (double, default: 2.0)
  - Time threshold for voxel updates

- **`min_rel_z`** (double, default: -1.5)
  - Minimum relative Z height for point processing

- **`max_rel_z`** (double, default: 0.2)
  - Maximum relative Z height for point processing

- **`dis_ratio_z`** (double, default: 0.2)
  - Distance ratio for Z filtering

## Usage

### Launch with Default Parameters

```bash
ros2 launch terrain_analysis terrain_analysis.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch terrain_analysis terrain_analysis.launch.py params_file:=path/to/custom_params.yaml
```

## Contributing

Please follow the existing code style and submit pull requests for any improvements.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Based on original work from [jizhang-cmu/autonomy_stack_mecanum_wheel_platform](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform)
