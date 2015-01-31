// project:     d_cv@tinker
// file:        writing_image.cpp
// created by bss at 2015-02-01
// Last modified: 2015-02-01, 01:07:46
// description: 

#include "ros/ros.h"
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "reading_image.h"

int main(int argc, char** argv)
{
    if (argc != 1)
    {
        printf("Error: please input file name.\n");
        printf("for example: rosrun d_cv test_writing_image test1\n");
        return 2;
    }
    for (int j = 0; j < strlen(argv[0]); j++)
    {
        if ('.' == argv[0][j])
        {
            printf("Error: invalid file name.\n");
            return 2;
        }
    }

    /* Init */
    printf("Begin init\n");

    ros::init(argc, argv, "test_reading_image");
    ros::NodeHandle n;
    ros::Rate rate(33);

    ImageConverter ic;
    cv::namedWindow("followme image window, test");

    printf("Waiting for ImageConverter\n");
    while (!ic.ready)
    {
        ros::spinOnce();
        rate.sleep();
        if (!ros::ok())
        {
            return 0;
        }
    }
    
    printf("Init OK\n");

    std::string package_path = ros::package::getPath("d_cv");
    std::string file_path = package_path +
            "/../../../share/d_cv/" + argv[0] + ".png";

    cv::imwrite(file_path.c_str(), ic.curr_image);

    printf("Save ok, Bye!\n");
    return 0;
}
