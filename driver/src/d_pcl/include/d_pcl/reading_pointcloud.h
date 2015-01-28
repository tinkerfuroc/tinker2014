// Project      : reading_pointcloud
// File         : reading_pointcloud.h
// Author       : bss
// Creation Date: 2014-07-12
// Last modified: 2014-07-12, 04:02:10
// Description  : convert ros-style pointcloud to pcl-style.
// 

#ifndef __READING_POINTCLOUD__
#define __READING_POINTCLOUD__

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class CloudConverter
{
public:
    CloudConverter();
    ~CloudConverter();

    void cloudXYZCb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

    void cloudXYZRGBCb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

public:
    bool ready_xyz_;
    bool ready_xyzrgb_;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr msg_xyz_;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr msg_xyzrgb_;

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_xyz_;
    ros::Subscriber sub_xyzrgb_;
};

#endif

