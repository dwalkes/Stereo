#ifndef __SEGMENT_H__
#define __SEGMENT_H__

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>

class Segment
{
public:
    int width, height;
    double mean;
    Segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointIndices indices);
    pcl::PointIndices indices;
    pcl::PointXY top, bottom, center;    
};

#endif
