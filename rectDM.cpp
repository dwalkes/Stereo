#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "depthmap.h"

cv::Mat** loadRMap(const char* intrinsics, const char* extrinsics, cv::Size imageSize)
{
    cv::FileStorage fsi(intrinsics, cv::FileStorage::READ);
    cv::Mat cameraMatrix[2], distCoeffs[2];
    fsi["M1"] >> cameraMatrix[0];
    fsi["M2"] >> cameraMatrix[1];
    fsi["D1"] >> distCoeffs[0];
    fsi["D2"] >> distCoeffs[1];
    cv::FileStorage fse(extrinsics, cv::FileStorage::READ);
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    fse["R"] >> R;
    fse["T"] >> T;
    fse["Q"] >> Q;
    fse["R1"] >> R1;
    fse["R2"] >> R2;
    fse["P1"] >> P1;
    fse["P2"] >> P2;
    cv::Mat** rmap = new cv::Mat*[2];
    rmap[0] = new cv::Mat[2];
    rmap[1] = new cv::Mat[2];
    cv::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    return rmap;
}

cv::Mat rectify(cv::Mat& source, cv::Mat** rmap, char index)
{
    cv::Mat rimg;
    cv::remap(source, rimg, rmap[index][0], rmap[index][1], CV_INTER_LINEAR);
    return rimg;
}

int main(int argc, char** argv)
{
    cv::Size imageSize(640, 480);
    cv::Mat** rmap = loadRMap(argv[1], argv[2], imageSize);
    
    cv::Mat img = cv::imread(argv[3], 0), rimg, cimg;
    cimg = rectify(img, rmap, 0);
    cv::imshow("l", cimg);
    
    img = cv::imread(argv[4], 0);
    cv::Mat cimg2 = rectify(img, rmap, 1);
    cv::imshow("r", cimg2);
    cv::Mat dmap = dm::normalize(dm::getDepthMapBM(cimg, cimg2));
    cv::imshow("d", dmap);
    cv::waitKey();

    return 0;
}        
