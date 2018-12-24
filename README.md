# Open3d_ROS

## About repo
This repository provide ROS integration with Intel(R) Open3D pointcloud library for further processing.

## How to use
The main function in the repository is `Open3D_ROS::fromROSMsg` which aims to resemble a similar function under pcl library, `pcl::fromROSMsg`. Note however that a copy of the header information is still not implemented yet.
A sample use of the `Open3D_ROS::fromROSMsg` is shown in the same class under the `Open3D_ROS::callback` function.

To run the sample node create, pass the cloud topic parameter as follows,
```
 rosrun open3d_ros open3d_ros_node cloud_topic:=/camera/depth/points
```
Or use the provided launch file after modifying the topic name.
```
roslaunch open3d_ros sample.launch
```