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
int tmp_sgmb_numberOfDisparities;
int sgbm_uniquenessRatio = 10;
int sgbm_speckleWindowSize = 50;
int sgbm_speckleRange = 36;

void save(const char* fname)
{
    cv::FileStorage fs(fname, CV_STORAGE_WRITE);
    fs << "preFilterCap" << sgbm_preFilterCap;
    fs << "SADWindowSize" << sgbm_SADWindowSize;
    fs << "minDisparity" << sgbm_minDisparity;
    fs << "P1" << sgbm_P1;
    fs << "P2" << sgbm_P2;
    fs << "numberOfDisparities" << sgmb_numberOfDisparities;
    fs << "uniquenessRatio" << sgbm_uniquenessRatio;
    fs << "speckleWindowSize" << sgbm_speckleWindowSize;
    fs << "speckleRange" << sgbm_speckleRange;
    fs.release();    
}

void load(const char* fname)
{
    cv::FileStorage fs(fname, CV_STORAGE_READ);
    fs ["preFilterCap"] >> sgbm_preFilterCap;
    fs ["SADWindowSize"] >> sgbm_SADWindowSize;
    fs ["minDisparity"] >> sgbm_minDisparity;
    fs ["P1"] >> sgbm_P1;
    fs ["P2"] >> sgbm_P2;
    fs ["numberOfDisparities"] >> sgmb_numberOfDisparities;
    fs ["uniquenessRatio"] >> sgbm_uniquenessRatio;
    fs ["speckleWindowSize"] >> sgbm_speckleWindowSize;
    fs ["speckleRange"] >> sgbm_speckleRange;
    fs.release();    
}

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
    //if(tmp_sgmb_numberOfDisparities % 16 == 0) sgmb_numberOfDisparities = tmp_sgmb_numberOfDisparities;
    sgmb_numberOfDisparities = tmp_sgmb_numberOfDisparities - tmp_sgmb_numberOfDisparities%16;

    sgbm.numberOfDisparities = sgmb_numberOfDisparities;//((left.cols/8) + 15) & -16;
    sgbm.uniquenessRatio = sgbm_uniquenessRatio;
    sgbm.speckleWindowSize = sgbm_speckleWindowSize;
    sgbm.speckleRange = sgbm_speckleRange;//36;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = false;

    sgbm(left, right, res);
    return res;
}

int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout << "USAGE ./depthmap <params.yaml> <left> <right> <out>";
        return 0;
    }
    left = cv::imread(argv[2], 0);
    right = cv::imread(argv[3], 0);
    sgmb_numberOfDisparities = ((left.cols/8) + 15) & -16;
    tmp_sgmb_numberOfDisparities = sgmb_numberOfDisparities;
    sgbm_P1 = 8*cn*sgbm_SADWindowSize*sgbm_SADWindowSize;
    sgbm_P2 = 32*cn*sgbm_SADWindowSize*sgbm_SADWindowSize;
    load(argv[1]);
    cv::Mat dm = normalize(getDepthMapSGBM(left, right));
    cv::imwrite(argv[4], dm);
    return 0;
}

