/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <histo.h>
#include <ros/ros.h>
#include <ros/package.h>


// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

double threshold = 0.2;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
    cout << "*******************************************************" 
            << std::endl;
    cout << "Ground based people detection app options:" 
            << std::endl;
    cout << "   --help    <show_this_help>" << std::endl;
    cout << "   --svm     <path_to_svm_file>" << std::endl;
    cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>"
            << std::endl;
    cout << "   --min_h   <minimum_person_height (default = 1.3)>" 
            << std::endl;
    cout << "   --max_h   <maximum_person_height (default = 2.3)>" 
            << std::endl;
    cout << "*******************************************************"
            << std::endl;
    return 0;
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud,
        PointCloudT::Ptr& cloud, bool* new_cloud_available_flag)
{
    cloud_mutex.lock ();    
    // for not overwriting the point cloud from another thread
    *cloud = *callback_cloud;
    *new_cloud_available_flag = true;
    cloud_mutex.unlock ();
}

struct callback_args{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback (
        const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex () == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (
            data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red,
            "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
            "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " 
            << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
    if(pcl::console::find_switch (argc, argv, "--help") ||
            pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

    // package path
    std::string package_path = ros::package::getPath("detect_people");

    // Algorithm parameters:
    std::string svm_filename = package_path +
            "/trainedLinearSVMForPeopleDetectionWithHOG.yaml";
    float min_confidence = -1.5;
    float min_height = 1.3;
    float max_height = 2.3;
    float voxel_size = 0.06;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    // Kinect RGB camera intrinsics
    rgb_intrinsics_matrix 
            << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; 

    // Read if some parameters are passed from command line:
    pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
    pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
    pcl::console::parse_argument (argc, argv, "--min_h", min_height);
    pcl::console::parse_argument (argc, argv, "--max_h", max_height);

    // Read Kinect live stream:
    PointCloudT::Ptr cloud (new PointCloudT);

    // Read pcd files from the harddisk:
    int i = 0;
    std::stringstream id;
    id << i;
    std::string pcd_path = package_path + "/../../../share/d_pcl/";
    std::string pcd_folder = "1";
    std::string file_path = pcd_path + pcd_folder + "/pcd" 
            + id.str() + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path.c_str(), *cloud)
            == -1 )
    {
        printf("Error: no valid file in the dir.\n");
        return 0;
    }
    else
    {
        printf("load complete!\n");
    }

    // Display pointcloud:
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
    viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." 
            << std::endl;

    // Spin until 'Q' is pressed:
    viewer.spin();
    std::cout << "done." << std::endl;

    cloud_mutex.unlock ();

    // Ground plane estimation:
    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
        clicked_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(
            clicked_points_indices,ground_coeffs);
    std::cout << "Ground plane: " << ground_coeffs(0) << " "
            << ground_coeffs(1) << " " << ground_coeffs(2) << " "
            << ground_coeffs(3) << std::endl;

    // Initialize new viewer:
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // viewer initialization
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Create classifier for people detection:
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);  // load trained SVM

    // People detection app initialization:
    
    // people detection object
    pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;
    // set the voxel size
    people_detector.setVoxelSize(voxel_size);
    // set RGB camera intrinsic parameters
    people_detector.setIntrinsics(rgb_intrinsics_matrix);
    // set person classifier
    people_detector.setClassifier(person_classifier);
    // set person classifier
    //people_detector.setHeightLimits(min_height, max_height);
    // set sensor orientation to vertical
    //people_detector.setSensorPortraitOrientation(true);

    // For timing:
    static unsigned count = 0;
    static double last = pcl::getTime ();
    PointCloudT::Ptr cloud_people (new PointCloudT);
    histo* sample = new histo(3 * 110);
    int pic_count = 0;
    float hist_dist_min = 0;
    std::vector<pcl::people::PersonCluster<PointT> >::iterator it_min;
    int k_min = 0;
    // Main loop:
    while (!viewer.wasStopped())
    {
        printf("while loop\n");
        i++;
        std::stringstream id;
        id  << i;
        std::string file_path = pcd_path + pcd_folder + "/pcd"
                + id.str() + ".pcd";
        std::cout << "loading " + file_path << std::endl;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(
                file_path.c_str(), *cloud) != -1)
        // if a new cloud is available
        {
            std::cout << "load file done!" << std::endl;
            // Perform people detection on the new cloud:
            
            // vector containing persons clusters
            std::vector<pcl::people::PersonCluster<PointT> > clusters;
            people_detector.setInputCloud(cloud);
            // set floor coefficients
            people_detector.setGround(ground_coeffs);
            // perform people detection
            people_detector.compute(clusters);

            // get updated floor coefficients
            ground_coeffs = people_detector.getGround();

            // Draw cloud and people bounding boxes in the viewer:
            viewer.removeAllPointClouds();
            viewer.removeAllShapes();
            pcl::visualization::PointCloudColorHandlerRGBField<PointT>
                    rgb(cloud);
            viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
            unsigned int k = 0;
            hist_dist_min = 2.0;
            for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
            {
                if(it->getPersonConfidence() > min_confidence)
                // draw only people with confidence above a threshold
                { 
                    // draw theoretical person bounding box in the PCL viewer:
                    //it->drawTBoundingBox(viewer, k);
                    //k++;
                    pcl::copyPointCloud(*cloud, it->getIndices(),
                            *cloud_people);
                    if (pic_count <= 10)
                    {
                        histo* tmp = sample->calc_histogram(cloud_people,
                                it->getMax()(1), it->getMin()(1));
                        *sample = *sample + *tmp;
                        pic_count++;
                    }
                    else if (pic_count == 11)
                    {
                        sample->normalize();
                        pic_count++;
                    }
                    else
                    {
                        histo* tmp = sample->calc_histogram(cloud_people,
                                it->getMax()(1), it->getMin()(1));
                        float dist = sample->histo_dist_sq(tmp);
                        std::cout << "the histogram distance is " 
                                << dist << std::endl;
                        if (dist < hist_dist_min)
                        {
                            hist_dist_min = dist;
                            it_min = it;
                            k_min = k;
                        }
                    }
                    k++;
                }
            }
            std::cout << k << " people found" << std::endl;
            std::cout << "spining" << std::endl;
            viewer.spinOnce();
            std::cout << "spining done" << std::endl;
            if (pic_count <= 10)
                std::cout << "the pic_count is less than 10" << std::endl;
            else if (hist_dist_min < threshold)
            {
                it_min->drawTBoundingBox(viewer, k_min);
                std::cout << "the best coff is " << hist_dist_min 
                        << std::endl;
            }
            else
            {
                std::cout 
                        << "can't find that person! and the best coff is "
                        << hist_dist_min << std::endl;
            }

            // Display average framerate:
            if (++count == 30)
            {
                double now = pcl::getTime ();
                std::cout << "Average framerate: "
                        << double(count)/double(now - last)
                        << " Hz" <<  std::endl;
                count = 0;
                last = now;
            }
        }
        else
        {
            std::cout << "load done! input to quit or load again"
                    << std::endl;
            char ch = getchar();
            if ('q' == ch || 'Q' == ch)
            {
                break;
            }
            else
            {
                i = 10;
            }
        }
    }

    return 0;
}

