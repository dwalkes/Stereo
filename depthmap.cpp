#include "depthmap.h"

namespace dm
{
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


    cv::Mat toGray(const cv::Mat& rgb_image)
    {
        cv::Mat res;
        cv::cvtColor(rgb_image, res, CV_RGB2GRAY);
        return res;
    }

    cv::Mat normalize(cv::Mat const &depth_map)
    {
        double min;
        double max;
        cv::minMaxIdx(depth_map, &min, &max);
        cv::Mat adjMap;
        cv::convertScaleAbs(depth_map, adjMap, 255 / max);
        return adjMap;
    }

    cv::Mat getDepthMapVar(cv::Mat const &left, cv::Mat const &right)
    {
        cv::Mat res;
        cv::StereoVar var;
        var.levels = 3; // ignored with USE_AUTO_PARAMS (!)
        var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
        var.nIt = 25;
        var.minDisp = ((left.cols/8) + 15) & -16;
        var.maxDisp = 0;
        //var.poly_n = 3;
        var.poly_n = 9;
        //var.poly_sigma = 0.0;
        var.poly_sigma = 1.7;
        var.fi = 15.0f;//nice
        var.lambda = 0.04f;
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
}
