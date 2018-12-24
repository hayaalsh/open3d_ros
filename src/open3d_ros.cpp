#include "open3d_ros/open3d_ros.h"

Open3D_ROS::Open3D_ROS(ros::NodeHandle& nodeHandle):
    nodeHandle_(nodeHandle)
{
    std::string cloud_topic;
    if (!nodeHandle_.getParam("cloud_topic", cloud_topic)){
        ROS_WARN("Missing/non-exisiting point cloud topic name");
        ros::requestShutdown();
    } 
    cloudSubscriber_ = nodeHandle_.subscribe(cloud_topic, 1, &Open3D_ROS::callback, this);
    ROS_INFO_STREAM("Subscribed to topic \"" << cloud_topic << "\".");
}

Open3D_ROS::~Open3D_ROS(){}

void Open3D_ROS::callback(const sensor_msgs::PointCloud2ConstPtr& ros_cloud_ptr)
{
    ROS_INFO_STREAM("Received point cloud.");
    auto open3d_cloud_ptr = std::make_shared<open3d::PointCloud>();
    fromROSMsg(*ros_cloud_ptr, *open3d_cloud_ptr);
}

void Open3D_ROS::fromROSMsg(const sensor_msgs::PointCloud2 &ros_cloud_const, open3d::PointCloud &pointcloud)
{
    // create sensor_msgs::PointCloud2 iterators
    sensor_msgs::PointCloud2 ros_cloud = ros_cloud_const;
    sensor_msgs::PointCloud2Modifier pcd_modifier(ros_cloud);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float>   iter_xyz(ros_cloud,"x");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(ros_cloud, "rgb");

    // update open3d::PointCloud values
    pointcloud.Clear();
    for ( ; iter_xyz != iter_xyz.end(); ++iter_xyz, ++iter_rgb ){
        const float &x = iter_xyz[0];
        const float &y = iter_xyz[1];
        const float &z = iter_xyz[2];
        const float &r = iter_rgb[0];
        const float &g = iter_rgb[1];
        const float &b = iter_rgb[2];

        // Reject NAN points
        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(r) || std::isnan(g) || std::isnan(b)){
            ROS_WARN("rejected for nan in point(%f, %f, %f, %f, %f, %f)\n", x, y, z, r, g, b);
            continue;  
        }

        pointcloud.points_.push_back(Eigen::Vector3d(x, y, z));
        pointcloud.colors_.push_back(Eigen::Vector3d(r, g, b));
    }
    ROS_INFO_STREAM("Point cloud converted.");
}