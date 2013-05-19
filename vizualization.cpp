#include "vizualization.h"

//This function creates a PCL visualizer, sets the point cloud to view and returns a pointer
boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    return (viewer);
}

void showClustersSeparetly(const cv::Mat& img_rgb, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, std::vector<pcl::PointIndices> &clusters)
{
    for(int k = 0; k < clusters.size(); k++)
    {
        cv::Mat res = cv::Mat::zeros(img_rgb.rows, img_rgb.cols, CV_8U);
        //for(int i =0 ; i < point_cloud_ptr->points.size(); i++)
        for(int j =0 ; j < clusters[k].indices.size(); j++)
        {
            int i = clusters[k].indices[j];
            int x = point_cloud_ptr->at(i).x;
            int y = point_cloud_ptr->at(i).y;
            res.at<uchar>(y, x) = (int)(point_cloud_ptr->at(i).z);
        }
        cv::imshow("rec2", res);
        cv::waitKey();
    }

}

