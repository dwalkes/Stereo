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
int var_nIt = 25;
int var_minDisp;
int var_tmp_poly_n = 9;
int var_poly_n = 9;
int var_maxDisp = 0;
int var_polySigma = 17;
int var_fi = 30;
int var_lambda = 4;

cv::Mat getDepthMapVar(cv::Mat const &left, cv::Mat const &right)
{
    cv::Mat res;
    cv::StereoVar var;
    var.levels = 3; // ignored with USE_AUTO_PARAMS (!)
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    var.nIt = var_nIt; //25;
    var.minDisp = var_minDisp; //    
    var.maxDisp = var_maxDisp; //0;
    //var.poly_n = 3;
    var.poly_n = var_poly_n; //9;
    //var.poly_sigma = 0.0;
    var.poly_sigma = var_polySigma * 0.1; //1.7;
    var.fi =  var_fi/2.0;//15.0f;//nice
    var.lambda = var_lambda*0.01;//0.04f;
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
    var(left, right, res);
    return res;
}

cv::Mat getDepthMapSGBM(cv::Mat const &left, cv::Mat const &right)
{
    cv::Mat res;
    //cv::StereoSGBM sgbm(0, ((left.cols/8) + 15) & -16, 11, 0, 20);
    cv::StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    int SADWindowSize = 11;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = left.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = ((left.cols/8) + 15) & -16;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 50;
    sgbm.speckleRange = 36;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = false;

    sgbm(left, right, res);
    return res;
}

cv::Mat getDepthMapBM(cv::Mat const &left, cv::Mat const &right)
{
    cv::Mat res; 
    cv::StereoBM bm(CV_STEREO_BM_NORMALIZED_RESPONSE);
    //cv::StereoBM bm(CV_STEREO_BM_NARROW);
    bm(left, right, res);
    return res;
}

void on_trackbar( int val, void* param)
{
    if(var_tmp_poly_n % 2 == 1) var_poly_n = var_tmp_poly_n;
    cv::Mat dm = normalize(getDepthMapVar(left, right));
    cv::imshow("depth map", dm);
}


int main(int argc, char** argv)
{
    left = cv::imread(argv[1], 0);
    right = cv::imread(argv[2], 0);
    var_minDisp = ((left.cols/8) + 15) & -16;
    cv::Mat dm = normalize(getDepthMapVar(left, right));
    cv::imshow("depth map", dm);
    cv::createTrackbar( "nIt", "depth map", &var_nIt, 200, on_trackbar );
    cv::createTrackbar( "mindisp", "depth map", &var_minDisp, 200, on_trackbar );
    cv::createTrackbar( "maxDisp", "depth map", &var_maxDisp, 200, on_trackbar );
    cv::createTrackbar( "poly_n", "depth map", &var_tmp_poly_n, 20, on_trackbar );
    cv::createTrackbar( "poly_sigma", "depth map", &var_polySigma, 100, on_trackbar );
    cv::createTrackbar( "fi", "depth map", &var_fi, 100, on_trackbar );
    cv::createTrackbar( "lambda", "depth map", &var_lambda, 100, on_trackbar );
    while (1) cv::waitKey(0);
}
