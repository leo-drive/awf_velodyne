Repository for VelodyneLiDAR VLS sensors (It currently supports VLS-128).

The code is highly leveraged from https://github.com/ros-drivers/velodyne. 

This code is only tested in ROS 2 Humble.

Compile instruction:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Note: The packages in this repository depend on velodyne_msgs in the [velodyne repository](https://github.com/ros-drivers/velodyne/tree/humble-devel). Therefore, this repository might need the update if there are any updates in the dependent package. Currently, this repository is tested with v2.4.0 of the velodyne_msgs from commit hash [d8cf623a922b1f12995e8c71295924c2905bd9a3](https://github.com/ros-drivers/velodyne/commit/d8cf623a922b1f12995e8c71295924c2905bd9a3).
