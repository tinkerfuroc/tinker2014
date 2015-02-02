// Project      : d_pcl
// File         : writing_pcds.cpp
// Author       : bss
// Creation Date: 2015-01-29
// Last modified: 2015-02-02, 10:52:04
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
// analysis args
bool AnalysisOpts(int argc, char** argv,
        std::string& pcd_name, int& rate_Hz);
// check dir
bool IsValidDir(char* dir);

int count = 0;
// pcd file path
std::string pcd_path = "";
// ascii or binary
bool saveAsASCII = false;

int main(int argc, char** argv)
{
    printf("Init\n");

    std::string package_path = ros::package::getPath("d_pcl");
    std::string pcd_name = "";

    int rate_Hz = 5;

    // get opts
    if (!AnalysisOpts(argc, argv, pcd_name, rate_Hz))
    {
        return 2;
    }
    if ("" == pcd_name)
    {
        fprintf(stderr, "Error: must input a dir name.\n");
        Usage();
        return 2;
    }
    pcd_path = package_path + "/../../../share/d_pcl/"
            + pcd_name;


    // if dir exists
    std::string cmd;
    cmd = "[ -d \"" + pcd_path + "\" ]";
    printf("check: %s\n", pcd_name.c_str());
    bool dir_not_exist = system(cmd.c_str());
    fflush(stdout);
    if (dir_not_exist)    // not exist
    {
        fprintf(stderr, "dir do not exists, will create new dir.\n");
        cmd = "mkdir " + pcd_path;
        system(cmd.c_str());
    }
    else    // exist
    {
        fprintf(stderr, "dir already exists, clear it? [Y/n]");
        char ch = getchar();
        if ('Y' == ch || 'y' == ch || '\r' == ch || '\n' == ch)
        {
            printf("will clear dir.\n");
            cmd = "rm " + pcd_path + "/*.*";
            system(cmd.c_str());
        }
        else
        {
            printf("Bye!\n");
            return 0;
        }
    }
    fflush(stdout);
    
    // init ros
    ros::init(argc, argv, "test_writing_pcds");
    ros::NodeHandle n;
    ros::Rate rate(rate_Hz);
    // subscribe
    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(
            "/pcl/points2", 1, getCloudCb);

    // loop
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
}

bool AnalysisOpts(int argc, char** argv,
        std::string& pcd_name, int& rate_Hz)
{
    bool IsDirSet = false;

    if (argc <= 1)
    {
        Usage();
        return false;
    }
    for (int i = 1; i < argc; i++)  // 检查命令行参数
    {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            Usage();
        }
        else if (argv[i][0] != '-')
        {
            if (!IsDirSet && IsValidDir(argv[i]))
            {
                // store name
                pcd_name = argv[i];
            }
        }
        else if (strcmp(argv[i], "-d") == 0 ||
                strcmp(argv[i], "--dir") == 0)
        {
            ++i;
            if (IsDirSet)
            {
                printf("Ignore -d/--dir because dir param has been set.\n");
            }
            if (i == argc)
            {
                fprintf(stderr,
                        "Error: please input dir name after -d/--dir.\n");
                return false;
            }
            if (!IsValidDir(argv[i]))
            {
                printf("Invalid dir \"%s\".", argv[i]);
                return false;
            }
            // store name
            pcd_name = argv[i];
            IsDirSet = true;
        }
        else if (strcmp(argv[i], "-r") == 0 ||
                strcmp(argv[i], "--rate") == 0)
        {
            ++i;
            if (i == argc)
            {
                fprintf(stderr,
                        "Error: please input rate as a parameter.\n");
                return false;
            }
            rate_Hz = atoi(argv[i]);
        }
        else if (strcmp(argv[i], "--ascii") == 0)
        {
            saveAsASCII = true;
        }
    }
    return true;
}

bool IsValidDir(char* dir)
{
    int len = strlen(dir);
    // safety check
    for (size_t j = 0; j < len; j++)
    {
        if (dir[j] == '.' || dir[j] == '/')
        {
            return false;
        }
    }
    return true;
}

void Usage()
{
    printf("\n");
    printf("test_writing_pcds node in d_pcl.\n");
    printf("Usage: rosrun d_pcl test_writing_pcds ");
    printf("[OPTION] DEST\n");
    printf("指定一个名字(DEST)，将/pcl/points2保存到该目录下\n");
    printf("不要尝试过于不合法的输入\n");
    printf("DEST: output dir name(put it in ui/d_pcl).\n");
    printf("-h,--help: print help message.\n");
    printf("-r,--rate: sending rate, in Hz.\n");
    printf("--ascii: save pcd in ascii format, buggy and slow.\n");
    printf("\n");
    printf("example:\n");
    printf("rosrun d_pcl test_writing_pcds tests\n");
}

void getCloudCb(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    printf("Get %d pcl.\n", count);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    cloud.width = msg->width;    // looks bad
    cloud.height = msg->height;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.points.clear();
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        cloud.points.push_back(msg->points[i]);
    }

    // save the file
    std::stringstream ss;
    ss << pcd_path << "/pcd" << count++ << ".pcd";
    if (saveAsASCII)
    {
        pcl::io::savePCDFileASCII(ss.str().c_str(), cloud);
    }
    else
    {
        pcl::io::savePCDFileBinary(ss.str().c_str(), cloud);
    }
}

