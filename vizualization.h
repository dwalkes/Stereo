#ifndef __VIZUALIZATION_H__
#define __VIZUALIZATION_H__

#include <cv.h>
#include <highgui.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
void showClustersSeparetly(const cv::Mat& img_rgb, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, std::vector<pcl::PointIndices> &clusters);


#endif
