// Project      : reading_pointcloud
// File         : show_xyz.cpp
// Author       : bss
// Creation Date: 2014-07-12
// Last modified: 2015-01-28, 21:41:21
// Description  : show ros-style pointcloud.
// 

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <d_pcl/reading_pointcloud.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_sub_cloud_xyzrgb");
    ros::NodeHandle n;
    ros::Rate rate(13);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    CloudConverter* cc_ = new CloudConverter();

    while (!cc_->ready_xyzrgb_)
    {
        ros::spinOnce();
        rate.sleep();
        if (!ros::ok())
        {
            printf("Terminated by C-c.\n");
            return -1;
        }
    }

    viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    //viewer.setBackgroundColor(0, 0, 0);
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
    //viewer.addCoordinateSystem(1.0);
    //viewer.initCameraParameters();

    while (ros::ok())
    {
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();

        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr msg = cc_->msg_xyzrgb_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
        printf ("%s, Cloud: width = %d, height = %d\n", cloud->header.frame_id.c_str(), cloud->width, cloud->height);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "input_cloud");

        viewer.spinOnce(20);
        ros::spinOnce();

        rate.sleep();
    }

    delete cc_;
    cc_ = NULL;

    return 0;
}
