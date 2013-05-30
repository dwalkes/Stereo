#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

cv::Mat normalize(cv::Mat const &depth_map)
{
    double min;
    double max;
    cv::minMaxIdx(depth_map, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(depth_map, adjMap, 255 / max);
    return adjMap;
}

cv::Mat left, right;
int sgbm_preFilterCap = 63;
int sgbm_SADWindowSize = 11;
int sgbm_minDisparity = 0;
int cn;
int sgbm_P1;
int sgbm_P2;
int sgmb_numberOfDisparities;
int sgbm_uniquenessRatio = 10;
int sgbm_speckleWindowSize = 50;
int sgbm_speckleRange = 36;

cv::Mat getDepthMapSGBM(cv::Mat const &left, cv::Mat const &right)
{
    cv::Mat res;
    cv::StereoSGBM sgbm;
    sgbm.preFilterCap = sgbm_preFilterCap; ///63;
    int SADWindowSize = sgbm_SADWindowSize; //11;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    cn = left.channels();

    sgbm.P1 = sgbm_P1;//8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = sgbm_P2;//32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = sgbm_minDisparity;//0;
    sgbm.numberOfDisparities = sgmb_numberOfDisparities;//((left.cols/8) + 15) & -16;
    sgbm.uniquenessRatio = sgbm_uniquenessRatio;
    sgbm.speckleWindowSize = sgbm_speckleWindowSize;
    sgbm.speckleRange = sgbm_speckleRange;//36;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = false;

    sgbm(left, right, res);
    return res;
}

void on_trackbar( int val, void* param)
{
    cv::Mat dm = normalize(getDepthMapSGBM(left, right));
    cv::imshow("depth map", dm);
}


int main(int argc, char** argv)
{
    left = cv::imread(argv[1], 0);
    right = cv::imread(argv[2], 0);
    sgmb_numberOfDisparities = ((left.cols/8) + 15) & -16;
    sgbm_P1 = 8*cn*sgbm_SADWindowSize*sgbm_SADWindowSize;
    sgbm_P2 = 32*cn*sgbm_SADWindowSize*sgbm_SADWindowSize;

    cv::Mat dm = normalize(getDepthMapSGBM(left, right));
    cv::imshow("depth map", dm);
    cv::createTrackbar( "preFilterCap", "depth map", &sgbm_preFilterCap, 200, on_trackbar );
    cv::createTrackbar( "SADWindowSize", "depth map", &sgbm_SADWindowSize, 200, on_trackbar );
    cv::createTrackbar( "P1", "depth map", &sgbm_P1, 300, on_trackbar );
    cv::createTrackbar( "P2", "depth map", &sgbm_P2, 300, on_trackbar );
    cv::createTrackbar( "minDisparity", "depth map", &sgbm_minDisparity, 60, on_trackbar );
    cv::createTrackbar( "numberOfDisparities", "depth map", &sgmb_numberOfDisparities, 100, on_trackbar );
    cv::createTrackbar( "sgbm_uniquenessRatio", "depth map", &sgbm_uniquenessRatio, 100, on_trackbar );
    cv::createTrackbar( "speckleWindowSize", "depth map", &sgbm_speckleWindowSize, 200, on_trackbar );
    cv::createTrackbar( "speckleRange", "depth map", &sgbm_speckleRange, 200, on_trackbar );
    while (1) cv::waitKey(0);
}
