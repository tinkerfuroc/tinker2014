// Project      : reading_pointcloud
// File         : writing_pcds.cpp
// Author       : bss
// Creation Date: 2015-01-29
// Last modified: 2015-01-29, 22:37:21
// Description  : show ros-style pointcloud.
// 

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// print usage of this node
void Usage();
// callback, when recv pointcloud
void getCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

int count = 0;
// pcd file path
std::string pcd_path = "";

int main(int argc, char** argv)
{
    printf("Init\n");

    std::string package_path = ros::package::getPath("d_pcl");
    std::string pcd_name = "";

    int rate_Hz = 5;

    if (argc <= 1)
    {
        Usage();
        return -1;
    }
    for (int i = 1; i < argc; i++)  // 检查命令行参数
    {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            Usage();
        }
        else if (argv[i][0] != '-')
        {
            pcd_name = argv[i];
            pcd_path = package_path + "/../../../share/d_pcl/"
                    + argv[i];
        }
        else if (strcmp(argv[i], "-r") == 0 ||
                strcmp(argv[i], "--rate") == 0)
        {
            ++i;
            if (i == argc)
            {
                printf("Error: please input rate as a parameter.\n");
                return -1;
            }
            else
            {
                rate_Hz = atoi(argv[i]);
            }
        }
    }
    
    // init ros
    ros::init(argc, argv, "test_writing_pcds");
    ros::NodeHandle n;
    ros::Rate rate(rate_Hz);
    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
            "/pcl/points2", 1, getCloudCb);

    // loop
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
}

void Usage()
{
    printf("test_writing_pcds node in d_pcl.\n");
    printf("Usage: rosrun d_pcl test_writing_pcds ");
    printf("[OPTION] DEST\n");
    printf("指定一个名字(DEST)，将/pcl/points2保存到该目录下\n");
    printf("不要尝试过于不合法的输入\n");
    printf("DEST: output dir name(put it in ui/d_pcl).\n");
    printf("-h,--help: print help message.\n");
    printf("-r,--rate: sending rate, in Hz.\n");
    printf("\n");
    printf("example:\n");
    printf("rosrun d_pcl test_writing_pcds tests\n");
}

void getCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    cloud.width = msg->width;
    cloud.height = msg->height;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        cloud.points.push_back(msg->points[i]);
    }

    // save the file
    std::stringstream ss;
    ss << pcd_path << "/pcd" << count++ << ".pcd";
    pcl::io::savePCDFileASCII(ss.str().c_str(), cloud);
}


