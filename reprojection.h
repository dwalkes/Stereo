#ifndef __REPROJECTION_H__
#define __REPROJECTION_H__
#include <cv.h>
#include <highgui.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

void straight_reproject_cloud(cv::Mat& img_rgb, cv::Mat& img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr);
void complex_reproject_cloud(const cv::Mat& Q, cv::Mat& img_rgb, cv::Mat& img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr);
#endif
