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
#include "segment.h"
#include "vizualization.h"

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
    reg.setMinClusterSize (500);
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

std::vector<Segment> getFilteredSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{
    std::vector<Segment> res;
    for(int i = 0; i < clusters.size(); i++)
    {
        res.push_back(Segment(cloud, clusters[i]));
    }
    return res;
}

std::vector<Segment> frameWork(cv::Mat& img_rgb, cv::Mat& right_img, cv::Mat (*getDepthMap)(const cv::Mat& left, const cv::Mat& right))
{
    //cv::Mat img_disparity = dm::getDepthMapVar(dm::toGray(img_rgb), dm::toGray(right_img)); 
    //cv::Mat img_disparity = dm::normalize(dm::getDepthMapBM(dm::toGray(img_rgb), dm::toGray(right_img))); 
    //cv::Mat img_disparity = dm::normalize(dm::getDepthMapSGBM(dm::toGray(img_rgb), dm::toGray(right_img))); 
    cv::Mat img_disparity = dm::normalize(getDepthMap(dm::toGray(img_rgb), dm::toGray(right_img))); 

    cv::imshow("rbg-image", img_rgb);
    cv::imshow("disparity-image", img_disparity);
    cv::waitKey(1);
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    //complex_reproject_cloud(Q, img_rgb, img_disparity, point_cloud_ptr); 
    straight_reproject_cloud(img_rgb, img_disparity, point_cloud_ptr); 

    auto reg = getColored(point_cloud_ptr);
    auto clusters = getClusters(reg);
    std::cout<<"Computing segments\n";
    auto segments = getFilteredSegments(point_cloud_ptr, clusters);
    showBoxes(img_rgb, segments);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    //showClustersSeparetly(img_rgb, point_cloud_ptr, clusters);

    point_cloud_ptr = reg.getColoredCloud();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( point_cloud_ptr );
  
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return segments;
}

void printHelp(char** argv)
{
    std::cerr << "Usage: " << argv[0] << " [-v] [-m var|bm|sgbm] -l <left image> -r <right image> " << std::endl;
    std::cerr << "\t-m mode" << std::endl;
    std::cerr << "\t-v vdeo" << std::endl;
}

int main( int argc, char** argv )
{
    char* left_name;
    char* right_name;
    bool isVideo = false;
    cv::Mat  (*getDM)(const cv::Mat& left, const cv::Mat& right) = &dm::getDepthMapSGBM;
    if (argc < 3)
    {
        printHelp(argv);
        return 1;
    }

    for(int i = 0; i < argc; i++)
    {
        if(strcmp(argv[i], "-l") == 0) left_name = argv[++i];
        if(strcmp(argv[i], "-r") == 0) right_name = argv[++i];
        if(strcmp(argv[i], "-v") == 0) isVideo = true;
        if(strcmp(argv[i], "-m") == 0)
        {
            i++;
            if(strcmp(argv[i], "bm") == 0) getDM = dm::getDepthMapBM;
            if(strcmp(argv[i], "sgbm") == 0) getDM = dm::getDepthMapSGBM;
            if(strcmp(argv[i], "var") == 0) getDM = dm::getDepthMapVar;
        }
    }

    cv::Mat img_rgb = cv::imread(left_name, CV_LOAD_IMAGE_COLOR);
    cv::Mat right_img = cv::imread(right_name, CV_LOAD_IMAGE_COLOR);
    frameWork(img_rgb, right_img, getDM);
    return 0;
}
