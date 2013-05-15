#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <time.h>

#include "depthmap.h"

#define CUSTOM_REPROJECT
/*** To understand the CUSTOM_REPROJECT code, please read Chapter 12 of the book
  Learning OpenCV: Computer Vision with the OpenCV Library. (Page 435) 
  I am using it because cv::reprojectImageTo3D is not giving me the expected
  results for some reason.
  
  If you want to use this program with cv::reprojectImageTo3D please comment
  the previous #define CUSTOM_REPROJECT and recompile.
    
***/

cv::Mat toGray(const cv::Mat& rgb_image)
{
    cv::Mat res;
    cv::cvtColor(rgb_image, res, CV_RGB2GRAY);
    return res;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColored(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (100);
  reg.setMaxClusterSize (20000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters, tmp_clusters;
  reg.extract (tmp_clusters);
  for(int i = 0; i < tmp_clusters.size(); i++)
  {
      if(tmp_clusters[i].indices.size() > 0)
        clusters.push_back(tmp_clusters[i]);
  }

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  //384 288
  /*for(int i = 0; i < to_show.rows; i++)
    for(int j = 0; j < to_show.cols/2; j++)
    {
        to_show.data[i*to_show.cols + j] = 200;
    }
  */
  //while (counter < 5 || counter > clusters[0].indices.size ())
  for(int j = 0; j < clusters.size(); j++)
  {
      cv::Mat to_show = cv::Mat::zeros(288, 384, cv::DataType<bool>::type);
      std::cout <<j << " " << clusters[j].indices.size() << std::endl;
      for(int i = 0; i < clusters[j].indices.size(); i++)
      {
        //std::cout << clusters[j].indices[i] << std::endl;
        //std::cout << cloud->at(clusters[j].indices[i])<< std:: endl;
        to_show.data[clusters[j].indices[i]] = 200;
        //to_show.at<uchar>(clusters[j].indices[i]/cloud->height, clusters[j].indices[i]%cloud->height) = 200;
        //to_show.at<bool>((int)(cloud->at(clusters[j].indices[i]).y), (int)(cloud->at(clusters[j].indices[i]).x)) = 200;
        counter++;
      }
      cv::imshow("", to_show);
      cv::waitKey();
  }
  
  //cv::Mat mat_header_with_disbled_reference_counting(cloud->height, cloud->width, CV_32F, (void*)&cloud->points[0]);
  //cv::imshow("", mat_header_with_disbled_reference_counting);
  //cv::waitKey();
  return reg.getColoredCloud();
}

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}

void reprojectCloud(const cv::Mat& Q, cv::Mat& img_rgb, cv::Mat& img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr)
{
#ifdef CUSTOM_REPROJECT
  //Get the interesting parameters from Q
  double Q03, Q13, Q23, Q32, Q33;
  Q03 = Q.at<double>(0,3);
  Q13 = Q.at<double>(1,3);
  Q23 = Q.at<double>(2,3);
  Q32 = Q.at<double>(3,2);
  Q33 = Q.at<double>(3,3);
  
  std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;
  
#endif  
 
#ifndef CUSTOM_REPROJECT
  //Create matrix that will contain 3D corrdinates of each pixel
  cv::Mat recons3D(img_disparity.size(), CV_32FC3);
  
  point_cloud_ptr->width = img_rgb.cols;
  point_cloud_ptr->height = img_rgb.rows;
  //Reproject image to 3D
  std::cout << "Reprojecting image to 3D..." << std::endl;
  cv::reprojectImageTo3D( img_disparity, recons3D, Q, false, CV_32F );
#endif  
  double px, py, pz;
  uchar pr, pg, pb;
  
  for (int i = 0; i < img_rgb.rows; i++)
  {
    uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
#ifdef CUSTOM_REPROJECT
    uchar* disp_ptr = img_disparity.ptr<uchar>(i);
#else
    double* recons_ptr = recons3D.ptr<double>(i);
#endif
    for (int j = 0; j < img_rgb.cols; j++)
    {
      //Get 3D coordinates
#ifdef CUSTOM_REPROJECT
      uchar d = disp_ptr[j];
      if ( d == 0 ) continue; //Discard bad pixels
      double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
      px = static_cast<double>(j) + Q03;
      py = static_cast<double>(i) + Q13;
      pz = Q23;
      
      px = px/pw;
      py = py/pw;
      pz = pz/pw;
#else
      px = recons_ptr[3*j];
      py = recons_ptr[3*j+1];
      pz = recons_ptr[3*j+2];
#endif
      //Get RGB info
      pb = rgb_ptr[3*j];
      pg = rgb_ptr[3*j+1];
      pr = rgb_ptr[3*j+2];
      //Insert info into point cloud structure
      pcl::PointXYZRGB point;
      point.x = px;
      point.y = py;
      point.z = pz;
      uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
              static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->push_back (point);
    }
  }
  //point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
  //point_cloud_ptr->height = 1;

}

int main( int argc, char** argv )
{
  //Check arguments
  if (argc != 4)
  {
    std::cerr << "Usage: " << argv[0] << " <path-to-Q-matrix> <left image> <right image> " << std::endl;
    return 1;
  }

  //Load Matrix Q
  cv::FileStorage fs(argv[1], cv::FileStorage::READ);
  cv::Mat Q;
  
  fs["Q"] >> Q;
  
  //If size of Q is not 4x4 exit
  if (Q.cols != 4 || Q.rows != 4)
  {
    std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
    return 1;
  }

  std::cout << "Read matrix in file " << argv[1] << std::endl;
  cv::Mat img_rgb = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
  cv::Mat right_img = cv::imread(argv[3], CV_LOAD_IMAGE_COLOR);
  if (img_rgb.data == NULL)
  {
    std::cerr << "ERROR: Could not read rgb-image: " << argv[2] << std::endl;
    return 1;
  }
  
  //Load disparity image
  //cv::Mat img_disparity = getDepthMapVar(toGray(img_rgb), toGray(right_img)); //cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
  //cv::Mat img_disparity = normalize(getDepthMapBM(toGray(img_rgb), toGray(right_img))); //cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat img_disparity = dm::normalize(dm::getDepthMapSGBM(toGray(img_rgb), toGray(right_img))); //cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
  
  //Show both images (for debug purposes)
  cv::namedWindow("rgb-image");
  cv::namedWindow("disparity-image");
  cv::imshow("rbg-image", img_rgb);
  cv::imshow("disparity-image", img_disparity);
  std::cout << "Press a key to continue..." << std::endl;
  cv::waitKey(1);
  //cv::destroyWindow("rgb-image");
  //cv::destroyWindow("disparity-image");
  //Create point cloud and fill it
  std::cout << "Creating Point Cloud..." <<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  reprojectCloud(Q, img_rgb, img_disparity, point_cloud_ptr); 

  clock_t begin, end;
  double time_spent;
  begin = clock();
  point_cloud_ptr = getColored(point_cloud_ptr);
  end = clock();
  std::cout <<"time elapsed"<< (double)(end - begin) / CLOCKS_PER_SEC <<std::endl;

  //Create visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = createVisualizer( point_cloud_ptr );
  
  //Main loop
  while ( !viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  
  return 0;
}
