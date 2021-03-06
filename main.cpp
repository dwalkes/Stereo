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

pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> getRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

std::vector<Segment> detectSegments(std::vector<Segment>& segments)
{
    std::vector<Segment> res;

    for(std::vector<Segment>::iterator it=segments.begin(); it != segments.end(); ++it)
    {
        Segment &s=*it;
        if(s.height*1.0/s.width > 1.3 && s.height*1.0/s.width < 1.6)
            res.push_back(s);
	}
    std::cout<<"Find "<< res.size()<<std::endl;
    return res;
}

double similarity(Segment& os, Segment& ns)
{
    //if(abs(os.center.x - ns.center.x) > os.width/2 || abs(os.center.y - ns.center.y) > os.height/2 ) return 10000.0;
    return (
        abs(ns.top.x-os.top.x)  + abs(ns.top.y-os.top.y) 
        +abs(ns.bottom.x-os.bottom.x)  + abs(ns.bottom.y-os.bottom.y) 
        +abs(ns.width-os.width) +abs(ns.height-os.height) 
        //+ abs(ns.indices.indices.size()-os.indices.indices.size())
        //+ abs(ns.mean-os.mean)
    );
}

std::vector<Segment> findPairs(std::vector<Segment>& oldS, std::vector<Segment>& newS)
{
    std::vector<Segment> res;
    for(std::vector<Segment>::iterator it=oldS.begin(); it != oldS.end(); ++it)
    {
        Segment &os=*it;
        Segment *best_seg = NULL;
        double best_err = 10000;
        for(int i = 0; i < newS.size(); i++)
        {
            double err = similarity(os, newS[i]);
            if((err < 200) && (err < best_err))
            {
                best_err = err;
                best_seg = &newS[i];
            }
        }
        if(best_seg != NULL)
            res.push_back(*best_seg);
    }
    return res;
}

std::vector<Segment> frameWork(cv::Mat& img_rgb, cv::Mat& right_img, cv::Mat (*getDepthMap)(const cv::Mat& left, const cv::Mat& right), bool showCloud=false, cv::Mat **rectMap=NULL )
{
	if(rectMap != NULL)
	{
		img_rgb = dm::rectify(img_rgb,rectMap,0);
		right_img = dm::rectify(right_img,rectMap,1);
	}
	cv::Mat img_disparity = dm::normalize(getDepthMap(dm::toGray(img_rgb), dm::toGray(right_img)));

    cv::imshow("rbg-image", img_rgb);
    cv::imshow("disparity-image", img_disparity);
    cv::waitKey(1);
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    //complex_reproject_cloud(Q, img_rgb, img_disparity, point_cloud_ptr); 
    straight_reproject_cloud(img_rgb, img_disparity, point_cloud_ptr); 
	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal>  reg = getRegionGrowing(point_cloud_ptr);
    std::vector <pcl::PointIndices> clusters = getClusters(reg);
    std::cout<<"Computing segments\n";
    std::vector<Segment> segments = getFilteredSegments(point_cloud_ptr, clusters);
    cv::Mat to_show = drawBoxes(img_rgb, segments, cv::Scalar(255, 0, 0));
    cv::imshow("boxes", to_show);
    cv::waitKey(10);
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    //showClustersSeparetly(img_rgb, point_cloud_ptr, clusters);

    point_cloud_ptr = reg.getColoredCloud();

    if(showCloud)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer = createVisualizer( point_cloud_ptr );
        while ( !viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    return segments;
}

/**
* Allocates a rectification map based on intrisic and extrinsic filenames
* @param rmap - location to write a pointer to the allocated cv::Mat array structure.  This pointer must be freed with delete [] when no longer in use
* @param intrinsic_file, extrinsic_file - intrinsic and extrinsic filenames
* @param imageSize the size of the image to be rectified
* @return true if no error occurred.  This could be because neither intrinsic nor exstrnsic file was specified, in which chase rmap will be NULL
*/
static bool allocateRectmap(cv::Mat*** rmap, const char *intrinsic_file, const char *extrinsic_file,  cv::Size imageSize)
{
	*rmap = NULL;
	if(intrinsic_file != NULL || extrinsic_file != NULL)
	{
		if(intrinsic_file == NULL || extrinsic_file == NULL)
		{
			std::cout << "Must specify intrinsic and extrinsic files together or specify neither\n";
			return false;
		}
		*rmap = dm::loadRMap(intrinsic_file, extrinsic_file, imageSize);
	}
	return true;
}

void videWork(const char *left_name, const char* right_name, cv::Mat(*getDM)(const cv::Mat& left, const cv::Mat& right))
{
    cv::VideoCapture Lcap(left_name);
    cv::VideoCapture Rcap(right_name);
    if (!Lcap.isOpened() || !Rcap.isOpened())
    {
        std::cout  << "Could not open reference " << left_name << std::endl;
        std::cout  << "Could not open reference " << right_name << std::endl;
        return ;
    }
    cv::Mat frame1, frame2;
    std::vector<Segment> oldS;
    while(1)
    {
        Lcap >> frame1;
        Rcap >> frame2;
        if (frame1.empty()) break;
        std::vector<Segment> segments = frameWork(frame1, frame2, getDM);
        if(oldS.size() == 0)
            oldS = detectSegments(segments);
        else
            oldS = findPairs(oldS, segments);
        cv::Mat to_show = drawBoxes(frame1, oldS, cv::Scalar(0, 255, 0));
        cv::imshow("olds", to_show);
        cv::waitKey(10);
        for(std::vector<Segment>::iterator it=oldS.begin(); it != oldS.end(); ++it)
        {
            Segment &s=*it;
            std::cout<<">>>>"<<s.top<<s.width<<" "<<s.height<<std::endl;
        }
    }
}

void printHelp(char** argv)
{
    std::cerr << "Usage: " << argv[0] << " [-v] [-m var|bm|sgbm] [-i intrinsic_file] [-e extrinsic_file] -l <left image> -r <right image> " << std::endl;
    std::cerr << "\t-m mode" << std::endl;
    std::cerr << "\t-v vdeo" << std::endl;
    std::cerr << "\t-i and -e arguments are intrinsic and extrinsic yml files created for this camera configuration by opencl stereo_calib example code" << std::endl;
}

int main( int argc, char** argv )
{
    const char* left_name = "";
    const char* right_name = "";
    const char* extrinsic_file = NULL;
    const char* intrinsic_file = NULL;

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
        if(strcmp(argv[i], "-i") == 0)
        {
            i++;
            intrinsic_file = argv[i];
        }
        if(strcmp(argv[i], "-e") == 0)
        {
            i++;
            extrinsic_file = argv[i];
        }

    }
    if(strlen(left_name) == 0 || strlen(right_name) == 0)
    {
        printHelp(argv);
        return 1;
    }

    if(!isVideo)
    {
        std::cout<<"Working with photo\n";
        cv::Mat img_rgb = cv::imread(left_name, CV_LOAD_IMAGE_COLOR);
        cv::Mat right_img = cv::imread(right_name, CV_LOAD_IMAGE_COLOR);
        cv::Mat **rectMap = NULL;
        if(!allocateRectmap(&rectMap,intrinsic_file,extrinsic_file,right_img.size()))
        {
            return 1;
        }
        frameWork(img_rgb, right_img, getDM, true, rectMap);
        if(rectMap != NULL)
        {
            delete[] rectMap;
        }
    }
    else
    {
        std::cout<<"Working with video\n";
        videWork(left_name, right_name, getDM);
    }
    return 0;
}
