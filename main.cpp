#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <time.h>

#include "depthmap.h"
#include "reprojection.h"

pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> getColored(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (40000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (12.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    return reg;
}

std::vector <pcl::PointIndices> getClusters(pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal>& reg)
{
    std::vector <pcl::PointIndices> clusters, tmp_clusters;
    reg.extract (tmp_clusters);
    for(int i = 0; i < tmp_clusters.size(); i++)
    {
        if(tmp_clusters[i].indices.size() > 0)
        clusters.push_back(tmp_clusters[i]);
    }
    return clusters;
}

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

int main( int argc, char** argv )
{
    //Check arguments
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <left image> <right image> " << std::endl;
        return 1;
    }

    cv::Mat img_rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat right_img = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    //cv::Mat img_disparity = dm::getDepthMapVar(dm::toGray(img_rgb), dm::toGray(right_img)); 
    //cv::Mat img_disparity = dm::normalize(dm::getDepthMapBM(dm::toGray(img_rgb), dm::toGray(right_img))); 
    cv::Mat img_disparity = dm::normalize(dm::getDepthMapSGBM(dm::toGray(img_rgb), dm::toGray(right_img))); 

    //Show both images (for debug purposes)
    cv::namedWindow("rgb-image");
    cv::namedWindow("disparity-image");
    cv::imshow("rbg-image", img_rgb);
    cv::imshow("disparity-image", img_disparity);
    std::cout << "Press a key to continue..." << std::endl;
    cv::waitKey(1);
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    //complex_reproject_cloud(Q, img_rgb, img_disparity, point_cloud_ptr); 
    straight_reproject_cloud(img_rgb, img_disparity, point_cloud_ptr); 

    clock_t begin, end;
    double time_spent;
    begin = clock();
    auto reg = getColored(point_cloud_ptr);
    auto clusters = getClusters(reg);
    end = clock();

    std::cout <<"(clustering) time elapsed"<< (double)(end - begin) / CLOCKS_PER_SEC <<std::endl;
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    int counter = 0;
    showClustersSeparetly(img_rgb, point_cloud_ptr, clusters);

    point_cloud_ptr = reg.getColoredCloud();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( point_cloud_ptr );
  
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        /*viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i = 0; i < clusters[counter].indices.size(); i++)
        {
            tmp_cloud->push_back(point_cloud_ptr->at(clusters[counter].indices[i]));
        }
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(tmp_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (tmp_cloud, rgb, "reconstruction");
        counter++;
        if(counter >= clusters.size()) counter = 0;*/
    }

    return 0;
}
