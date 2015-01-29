// Project      : d_pcl
// File         : reading_pcd.cpp
// Author       : bss
// Creation Date: 2015-01-29
// Last modified: 2015-01-30, 00:33:38
// Description  : read pcd from file
// 

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// print usage of this node
void Usage();

int main(int argc, char** argv)
{
    printf("Init\n");

    std::string package_path = ros::package::getPath("d_pcl");
    // pcd file path
    std::string pcd_path = "";
    std::string pcd_name = "";

    int rate_Hz = 5;    // seems slow
    std::string repeat_func = "key";
    int repeat_times = 20;  // 延迟多少帧后重新发

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
        else if (strcmp(argv[i], "-f") == 0)
        {
            ++i;
            if (i == argc)
            {
                printf("Error: please select repeat function: ");
                printf("always,delay,key,not\n");
                return -1;
            }
            else
            {
                repeat_func = argv[i];
                if ("delay" == repeat_func)
                {
                    ++i;
                    if (i == argc)
                    {
                        printf("Error: please input repeat times,");
                        printf("for example: -f delay 10\n");
                        return -1;
                    }
                    else
                    {
                        repeat_times = atoi(argv[i]);
                    }
                }
            }
        }
        else
        {
            printf("Error: unknown parameter.\n");
            Usage();
            return -1;
        }
    }

    // init ros
    ros::init(argc, argv, "test_reading_pcds");
    ros::NodeHandle n;
    ros::Rate rate(rate_Hz);
    ros::Publisher pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
            "/pcl/points2", 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

    printf("Init ok\n");
    // publish topic
    for (int i = 0; ros::ok(); i++)
    {
        std::stringstream id;
        id << i;
        std::string file_path = pcd_path + "/pcd" + id.str() + ".pcd";
        // load the file
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path.c_str(),
                *cloud) == -1)
        {
            if (0 == i) // no file at all
            {
                printf("Error: no valid file in the dir.\n");
                break;
            }
            else
            {
                if ("key" == repeat_func)
                {
                    printf("Press a key to play again. q or C-c to quit.");
                    char ch = getchar();
                    if ('q' == ch || 'Q' == ch)
                    {
                        break;
                    }
                }
                else if ("delay" == repeat_func)
                {
                    for (size_t j = 0; ros::ok() && j < repeat_times; j++)
                    {
                        ros::spinOnce();
                        rate.sleep();
                    }
                }
                else if ("always" != repeat_func)
                {
                    break;  // don't repeat
                }
                // read again
                i = -1;  // i++ after continue
                continue;
            }
        }

        // read pcd
        cloud->header.frame_id = "test_pcd_" + pcd_name + "_" + id.str();

        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        pub.publish(cloud);
        ros::spinOnce();
        rate.sleep();
    }

    printf("Bye!\n");
    return 0;
}

void Usage()
{
    printf("\n");
    printf("test_reading_pcds node in d_pcl\n");
    printf("Usage: rosrun d_pcl test_reading_pcds \n");
    printf("[OPTION] SOURCE\n");
    printf("指定一个名字(SOURCE),读取/pcl/points2目录下到文件\n");
    printf("不要尝试过于不合法的输入\n");
    printf("SOURCE: input dir name(put it in ui/d_pcl).\n");
    printf("-h,--help: print help message.\n");
    printf("-r,--rate: sending rate, in Hz.\n");
    printf("-f: how to repeat.\n");
    printf("  always:always repeat;\n");
    printf("  delay times:延迟times帧再重复\n");
    printf("  key:wait for key;\n");
    printf("  [other]:don't repeat;\n");
    printf("  default is \'key\'\n");
    printf("\n");
    printf("example:\n");
    printf("rosrun d_pcl test_reading_pcds test\n");
}

