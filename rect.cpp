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

int main(int argc, char** argv)
{
    cv::Size imageSize(640, 480);
    //cv::Mat rmap[2][2];
    //cv::Mat** rmap = loadRMap("intrinsics.yml", "extrinsics.yml", imageSize);
    cv::Mat** rmap = loadRMap(argv[1], argv[2], imageSize);
    
    cv::Mat img = cv::imread("chess_photo/left0.jpg", 0), rimg, cimg;
    cv::remap(img, rimg, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
    cv::cvtColor(rimg, cimg, CV_GRAY2BGR);
    cv::imshow("l", cimg);
    cv::imwrite("l.jpg", cimg);
    
    img = cv::imread("chess_photo/right0.jpg", 0);
    cv::remap(img, rimg, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
    cv::cvtColor(rimg, cimg, CV_GRAY2BGR);
    cv::imshow("r", cimg);
    cv::imwrite("r.jpg", cimg);
    cv::waitKey();

    return 0;
}        
