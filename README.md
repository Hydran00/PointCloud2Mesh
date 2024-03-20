# PointCloud2Mesh
![Build Humble](https://github.com/Hydran00/PointCloud2Mesh/actions/workflows/main.yml/badge.svg)

ROS2 Package that compute the 3d mesh starting from a point cloud using `open3d` library.
## Requirements
- Ubuntu 22.04
- ROS2 Humble
## Dependencies
- Open3D
  ```
  pip install open3d
  ```
- Open3D ROS conversion package
  ```
  sudo apt-get install ros-humble-open3d-conversions
  ```
## Install
1. Clone this repo in your `src` folder inside your ROS2 workspace
2. Build
   ```
   colcon build --symlink-install
   ```
4. Source ROS2
   ```
   source install/setup.bash
   ```
5. Run the node
   ```
   ros2 run 3d_rec rec
   ```