#include "open3d_ros/open3d_ros.h"

/**
 * main function
 * **/
int main (int argc, char** argv)
{
  ros::init(argc, argv, "open3d_ros_node");
  ros::NodeHandle nh("~");
  Open3D_ROS pointcloud(nh);
  ros::spin();
  return 0;
}
