#include "reprojection.h"
void straight_reproject_cloud(cv::Mat& img_rgb, cv::Mat& img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr)
{
    uchar pr, pg, pb;
    for (int i = 0; i < img_rgb.rows; i++)
    {
        uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
        uchar* disp_ptr = img_disparity.ptr<uchar>(i);
        for (int j = 0; j < img_rgb.cols; j++)
        {
            uchar d = disp_ptr[j];
            if ( d == 0 ) continue; //Discard bad pixels
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];
            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = j;
            point.y = i;
            point.z = d;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                  static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->push_back (point);
        } 
    }
}

void complex_reproject_cloud(const cv::Mat& Q, cv::Mat& img_rgb, cv::Mat& img_disparity, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr)
{
    //Get the interesting parameters from Q
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

    std::cout << "Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;

    double px, py, pz;
    uchar pr, pg, pb;

    for (int i = 0; i < img_rgb.rows; i++)
    {
        uchar* rgb_ptr = img_rgb.ptr<uchar>(i);
        uchar* disp_ptr = img_disparity.ptr<uchar>(i);
        for (int j = 0; j < img_rgb.cols; j++)
        {
            //Get 3D coordinates
            uchar d = disp_ptr[j];
            if ( d == 0 ) continue; //Discard bad pixels
            double pw = -1.0 * static_cast<double>(d) * Q32 + Q33; 
            px = static_cast<double>(j) + Q03;
            py = static_cast<double>(i) + Q13;
            pz = Q23;

            px = px/pw;
            py = py/pw;
            pz = pz/pw;
            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];
            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                  static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->push_back (point);
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

}

