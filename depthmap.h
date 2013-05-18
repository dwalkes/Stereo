#ifndef __DEPTH_MAP_H__
#define __DEPTH_MAP_H__

#include <cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

namespace dm
{
    cv::Mat normalize(cv::Mat const &depth_map);
    cv::Mat getDepthMapSGBM(cv::Mat const &left, cv::Mat const &right);
    cv::Mat getDepthMapVar(cv::Mat const &left, cv::Mat const &right);
    cv::Mat getDepthMapBM(cv::Mat const &left, cv::Mat const &right);
    cv::Mat toGray(const cv::Mat& rgb_image);
}

#endif

