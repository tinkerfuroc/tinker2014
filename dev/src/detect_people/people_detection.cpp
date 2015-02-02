

#include <opencv2/opencv.hpp>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "frmsg/people.h"
#include "frmsg/followme_state.h"
#include <reading_pointcloud/reading_pointcloud.h>
#include <ros/time.h>
//#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NH 10
#define NS 10
#define NV 10
/* max HSV values */
#define H_MAX 360.0
#define S_MAX 1.0
#define V_MAX 1.0

/* low thresholds on saturation and value for histogramming */
#define S_THRESH 0.1
#define V_THRESH 0.2

int MIN_cmp( int a, int b )
{
  return a>b?b:a;
}

int max( int a, int b, int c )
{
  if ( a > b )
    return a>c?a:c;
  else
    return b>c?b:c;
}

int min( int a, int b, int c )
{
  if ( a < b )
    return a<c?a:c;
  else
    return b<c?b:c;
}

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

enum { COLS = 640, ROWS = 480 };

typedef struct histogram {
  float histo[NH*NS + NV];   /**< histogram array */
  int n;                     /**< length of histogram array */
}histogram;

int print_help()
{
  cout << "*******************************************************" << std::endl;
  cout << "Ground based people detection app options:" << std::endl;
  cout << "   --help    <show_this_help>" << std::endl;
  cout << "   --svm     <path_to_svm_file>" << std::endl;
  cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  cout << "*******************************************************" << std::endl;
  return 0;
}

int histo_bin( float h, float s, float v )
{
  int hd, sd, vd;

  /* if S or V is less than its threshold, return a "colorless" bin */
  vd = MIN_cmp( (int)(v * NV / V_MAX), NV-1 );
  if( s < S_THRESH  ||  v < V_THRESH )
    return NH * NS + vd;

  /* otherwise determine "colorful" bin */
  hd = MIN_cmp( (int)(h * NH / H_MAX), NH-1 );
  sd = MIN_cmp( (int)(s * NS / S_MAX), NS-1 );
  return sd * NH + hd;
}

void rgb2hsv( int r, int g, int b, float& h, float& s, float& v )
{
  int max_ = max( r, g, b );
  int min_ = min( r, g, b );
  v = max_;
  if ( max == 0 )
    s = 0;
  else
  s = ( max_ - min_ ) / (float)max_;
  if ( max_ == min_ )
    h = 0;
  if ( r == max_ )
    h = ( g - b ) / (float)( max_ - min_ ) * 60;
  if ( g == max_ )
    h = 120 + ( b - r ) / (float)( max_ - min_ ) * 60;
  if ( b == max_ )
    h = 240 + ( r - g ) / (float)( max_ - min_ ) * 60;
  if ( h < 0 )
    h += 360;
}

void normalize_histogram( histogram* histo )
{
  float* hist;
  float sum = 0, inv_sum;
  int i, n;

  hist = histo->histo;
  n = histo->n;

  /* compute sum of all bins and multiply each bin by the sum's inverse */
  for( i = 0; i < n; i++ )
    sum += hist[i];
  inv_sum = 1.0 / sum;
  for( i = 0; i < n; i++ )
    hist[i] *= inv_sum;
}

histogram* calc_histogram( PointCloudT::Ptr& cloud )
{
  histogram* histo;
  float* hist;
  int bin;
  float h = 0, s = 0, v = 0;

  histo = (histogram*)malloc( sizeof(histogram) );
  histo->n = NH*NS + NV;
  hist = histo->histo;
  memset( hist, 0, histo->n * sizeof(float) );

  for( int i = 0; i < cloud->points.size(); i++ )
  {
    rgb2hsv( (int)cloud->points[i].r, (int)cloud->points[i].g, (int)cloud->points[i].b, h, s, v );
  bin = histo_bin( h, s, v );
  hist[bin] += 1;
  }
  normalize_histogram( histo );
  return histo;
}

histogram* calc_histogram_a( PointCloudT::Ptr& cloud )
{
  histogram* histo;
  float* hist;
  int bin;
  float h = 0, s = 0, v = 0;

  histo = (histogram*)malloc( sizeof(histogram) );
  histo->n = NH*NS + NV;
  hist = histo->histo;
  memset( hist, 0, histo->n * sizeof(float) );

  for( int i = 0; i < cloud->points.size(); i++ )
  {
    rgb2hsv( (int)cloud->points[i].r, (int)cloud->points[i].g, (int)cloud->points[i].b, h, s, v );
  bin = histo_bin( h, s, v );
  hist[bin] += 1;
  }
  //normalize_histogram( histo );
  return histo;
}

float histo_dist_sq( histogram* h1, histogram* h2 )
{
  float* hist1, * hist2;
  float sum = 0;
  int i, n;

  n = h1->n;
  hist1 = h1->histo;
  hist2 = h2->histo;

  /*
    According the the Battacharyya similarity coefficient,

    D = \sqrt{ 1 - \sum_1^n{ \sqrt{ h_1(i) * h_2(i) } } }
  */
  for( i = 0; i < n; i++ )
    sum += sqrt( hist1[i]*hist2[i] );
  return 1.0 - sum;
}

void add_hist( histogram* hist1, histogram* hist2 )
{
  for ( int i = 0; i <  hist1->n; i++ )
  {
    hist1->histo[i] += hist2->histo[i];
  }
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

PointCloudT::Ptr& person_head(PointCloudT::Ptr& cloud)
{
	PointCloudT::Ptr head(new PointCloudT(*cloud));
	int max_z,min_z;
	max_z=min_z=cloud->points[1].z;
	for(int i=0;i<cloud->points.size();i++)
	{
		if(((int)cloud->points[i].z)>max_z)
		{max_z=(int)cloud->points[i].z;}
		if(((int)cloud->points[i].z)<min_z)
		{min_z=(int)cloud->points[i].z;}
	}
	int top_z,bottom_z;
	top_z=max_z;
	bottom_z=max_z-(int)((max_z-min_z)/8);

	int j=0;
	for(int i=0;i<(int)(cloud->points.size());i++)
	{
		if((cloud->points[i].z)>bottom_z&&(cloud->points[i].z)<top_z)
		{
			head->points[j]=cloud->points[i];
			j++;
		}
	}
	head->points.resize(j);
	return head;
}


PointCloudT::Ptr& person_body(PointCloudT::Ptr& cloud)
{
	PointCloudT::Ptr body(new PointCloudT(*cloud));
	int max_z,min_z;
	max_z=min_z=cloud->points[1].z;
	for(int i=0;i<cloud->points.size();i++)
	{
		if(((int)cloud->points[i].z)>max_z)
		{max_z=(int)cloud->points[i].z;}
		if(((int)cloud->points[i].z)<min_z)
		{min_z=(int)cloud->points[i].z;}
	}
	int top_z,bottom_z;
	top_z=max_z-(int)((max_z-min_z)/8);
	bottom_z=max_z-(int)((max_z-min_z)/2);

	int j=0;
	for(int i=0;i<(int)(cloud->points.size());i++)
	{
		if((cloud->points[i].z)>bottom_z&&(cloud->points[i].z)<top_z)
		{
			body->points[j]=cloud->points[i];
			j++;
		}
	}
	body->points.resize(j);
	return body;
}

PointCloudT::Ptr& person_leg(PointCloudT::Ptr& cloud)
{
	PointCloudT::Ptr leg(new PointCloudT(*cloud));
	int max_z,min_z;
	max_z=min_z=cloud->points[1].z;
	for(int i=0;i<cloud->points.size();i++)
	{
		if(((int)cloud->points[i].z)>max_z)
		{max_z=(int)cloud->points[i].z;}
		if(((int)cloud->points[i].z)<min_z)
		{min_z=(int)cloud->points[i].z;}
	}
	int top_z,bottom_z;
	top_z=max_z-(int)((max_z-min_z)/2);
	bottom_z=min_z;

	int j=0;
	for(int i=0;i<(int)(cloud->points.size());i++)
	{
		if((cloud->points[i].z)>bottom_z&&(cloud->points[i].z)<top_z)
		{
			leg->points[j]=cloud->points[i];
			j++;
		}
	}
	leg->points.resize(j);
	return leg;
}


int main (int argc, char** argv)
{

  //ROS Initialization
  ros::init(argc, argv, "detecting_people");
  ros::NodeHandle nh;
  ros::Rate rate(13);

  ros::Subscriber state_sub = nh.subscribe("followme_state", 5, &stateCallback);
  ros::Publisher people_pub = nh.advertise<frmsg::people>("followme_people", 5);
  frmsg::people pub_people_;

  CloudConverter* cc_ = new CloudConverter();

  while (!cc_->ready_xyzrgb_)
  {
    ros::spinOnce();
    rate.sleep();
    if (!ros::ok())
    {
      printf("Terminated by Control-c.\n");
      return -1;
    }
  }

  // Input parameter from the .yaml
  std::string package_path_ = ros::package::getPath("detecting_people") + "/";
  cv::FileStorage* fs_ = new cv::FileStorage(package_path_ + "parameters.yml", cv::FileStorage::READ);

  // Algorithm parameters:
  std::string svm_filename = package_path_ + "trainedLinearSVMForPeopleDetectionWithHOG.yaml";
  std::cout << svm_filename << std::endl;

  float min_confidence = -1.5;
  float min_height = 1.3;
  float max_height = 2.3;
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument (argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument (argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument (argc, argv, "--min_h", min_height);
  pcl::console::parse_argument (argc, argv, "--max_h", max_height);


  // Read Kinect live stream:
  PointCloudT::Ptr cloud_people (new PointCloudT);
  cc_->ready_xyzrgb_ = false;
  while ( !cc_->ready_xyzrgb_ )
  {
    ros::spinOnce();
    rate.sleep();
  }
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = cc_->msg_xyzrgb_;

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
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << "done." << std::endl;

  //cloud_mutex.unlock ();

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
//  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime ();

  int people_count = 0;
  histogram* first_hist;

  int  max_people_num = (int)fs_->getFirstTopLevelNode()["max_people_num"];

  // Main loop:
  while (!viewer.wasStopped() && ros::ok() )
  {
    if (cc_->ready_xyzrgb_)    // if a new cloud is available
    {
      cloud = cc_->msg_xyzrgb_;

      PointCloudT::Ptr cloud_new(new PointCloudT(*cloud));

      cc_->ready_xyzrgb_ = false;
      // Perform people detection on thw cloud:
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud_new);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection

      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
      viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
      unsigned int k = 0;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
          pcl::copyPointCloud(*cloud, it->getIndices(), *cloud_people);
          PointCloudT::Ptr head = person_head(cloud_people);
          PointCloudT::Ptr body = person_body(cloud_people);
          PointCloudT::Ptr leg = person_leg(cloud_people);
	  std::cout << "inhere" << std::endl;
	  std::cout << leg << std::endl;
        }
      }
      std::cout << k << " people found" << std::endl;
      viewer.spinOnce();

      // Display average framerate:
      if (++count == 30)
      {
        double now = pcl::getTime ();
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }
      //cloud_mutex.unlock ();

      ros::spinOnce();
    }
  }

  return 0;
}
