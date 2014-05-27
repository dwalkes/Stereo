/*
    CVStereoRectify.h
    Rectify a pair of stereo images using intrinsic and extrinsic camera parameters.
    Copyright Dan Walkes, Trellis-Logic LLC 2014    
    BSD License copied from opencv project license.dxt

    By downloading, copying, installing or using the software you agree to this license.
    If you do not agree to this license, do not download, install,
    copy or use the software.


                              License Agreement
                   For Open Source Computer Vision Library
                           (3-clause BSD License)

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

      * Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

      * Neither the names of the copyright holders nor the names of the contributors
        may be used to endorse or promote products derived from this software
        without specific prior written permission.

    This software is provided by the copyright holders and contributors "as is" and
    any express or implied warranties, including, but not limited to, the implied
    warranties of merchantability and fitness for a particular purpose are disclaimed.
    In no event shall copyright holders or contributors be liable for any direct,
    indirect, incidental, special, exemplary, or consequential damages
    (including, but not limited to, procurement of substitute goods or services;
    loss of use, data, or profits; or business interruption) however caused
    and on any theory of liability, whether in contract, strict liability,
    or tort (including negligence or otherwise) arising in any way out of
    the use of this software, even if advised of the possibility of such damage.
*/
#include <ostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class CVStereoRectify
{
private:
    std::ostream &m_Logger;
    /**
    * See validPixROIx parameters in stereoRectify
    */
    cv::Rect m_Roi1;
    cv::Rect m_Roi2;
    /**
    * See Q parameters in stereoRectify()
    */
    cv::Mat  m_DisparityToDepthMap;
    const char *m_IntrinsicFile;
    const char *m_ExtrinsicFile;
    cv::Size m_LastImageSize;
    struct UndistortRectifyMaps
    {
        cv::Mat map11;
        cv::Mat map12;
        cv::Mat map21;
        cv::Mat map22;
    }; 
    struct UndistortRectifyMaps m_LastRectMap;
    
    /**
    * @param img_size - the size of the image to compute undistort rectification maps
    * @return a pointer to the struct UndistortRectifyMaps corresponding to this image size or NULL if these values could not be computed
    */
    inline struct UndistortRectifyMaps *GetUndistortRectifyMaps(cv::Size img_size)
    {
        struct UndistortRectifyMaps *returnMap = NULL;
        if( img_size == m_LastImageSize )
        {
            returnMap = &m_LastRectMap;
        }
        else
        {
            cv::FileStorage fsIntrinsic(m_IntrinsicFile, CV_STORAGE_READ);
            cv::FileStorage fsExtrinsic(m_ExtrinsicFile, CV_STORAGE_READ);
            bool success = true;
            if( !fsIntrinsic.isOpened() )
            {
                m_Logger << "Failed to open Intrinsic file " << m_IntrinsicFile << std::endl;
                success = false;
            }
            else if( !fsExtrinsic.isOpened() )
            {
                m_Logger << "Failed to open Extrinsic file " << m_ExtrinsicFile << std::endl;
                success = false;
            }
            // TODO: add file validation        
            
            cv::Mat M1, D1, M2, D2;
            fsIntrinsic["M1"] >> M1;
            fsIntrinsic["D1"] >> D1;
            fsIntrinsic["M2"] >> M2;
            fsIntrinsic["D2"] >> D2;

            cv::Mat R, T, R1, P1, R2, P2;
            fsExtrinsic["R"] >> R;
            fsExtrinsic["T"] >> T;
            cv::stereoRectify( M1, D2, M2, D2, img_size, R, T, R1, R2, P1, P2, m_DisparityToDepthMap, cv::CALIB_ZERO_DISPARITY, -1, img_size, &m_Roi1, &m_Roi2 );
            cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, m_LastRectMap.map11, m_LastRectMap.map12);
            cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, m_LastRectMap.map21, m_LastRectMap.map22);
            if( success )
            {
                m_LastImageSize = img_size;
                returnMap = &m_LastRectMap;
            } 
        }
        return returnMap;
    }

public:
    CVStereoRectify():
        m_Logger(std::cout),
        m_IntrinsicFile(NULL),
        m_ExtrinsicFile(NULL)
    {
    }
    /*
    * Setup intrinsics/entrinsic files for stereo rectification
    */
    inline void setFiles(const char *intrinsic_filename, const char *extrinsic_filename)
    {
        m_IntrinsicFile=intrinsic_filename;
        m_ExtrinsicFile=extrinsic_filename;
    }

    /**
    * @return true if the class is transforming images due to provided intrinsic/extrinsic files
    */
    inline bool isTransforming()
    {
        if( m_IntrinsicFile == NULL || m_ExtrinsicFile == NULL )
        {
           return false;
        } 
        return true;
    }
  
    /*
    * Perform the stareo rectification step with previously provided intrinsic/extrinsic files.
    * Note: If no files have been performed this function is a no-op
    * @return true if the transform was performed or if !isTransforming(), false if an error occurred
    */ 
    inline bool transform(cv::Mat &img1, cv::Mat &img2)
    {
        if( !isTransforming() )
        {
            return true;
        }
        bool success = true; 
        if( img1.size() != img2.size() )
        {
            m_Logger << "Image and and Image 2 have different sizes of " << img1.size() << " and " << img2.size() << " respectively" << std::endl;
            success = false;
        }

        if( success )
        {
            struct UndistortRectifyMaps *returnMap=GetUndistortRectifyMaps(img1.size());
            if (returnMap != NULL)
            {
                cv::Mat img1r, img2r;
                cv::remap(img1, img1r, returnMap->map11, returnMap->map12, cv::INTER_LINEAR);
                cv::remap(img2, img2r, returnMap->map21, returnMap->map22, cv::INTER_LINEAR);
                img1 = img1r;
                img2 = img2r;
            }
            else
            {
                success = false;
            }
        }
        return success;
    }

    inline cv::Rect getROI1() { return m_Roi1; }
    inline cv::Rect getROI2() { return m_Roi2; }
};
