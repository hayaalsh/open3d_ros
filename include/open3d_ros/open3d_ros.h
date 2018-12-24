#ifndef OPEN3D_ROS_H
#define OPEN3D_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iostream>
#include <memory>
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

class Open3D_ROS
{
    public:
        Open3D_ROS(ros::NodeHandle& nodeHandle);
        virtual ~Open3D_ROS();
        void fromROSMsg(const sensor_msgs::PointCloud2 &ros_cloud, open3d::PointCloud &pointcloud);

    private:
        void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
        ros::NodeHandle& nodeHandle_;
        ros::Subscriber cloudSubscriber_;
};

#endif