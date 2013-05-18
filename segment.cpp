#include "segment.h"
#include <algorithm> 

Segment::Segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointIndices ind)
{
    this->indices = ind;
    this->top.x = 100000;
    this->top.y = 100000;
    this->bottom.x = -100000;
    this->bottom.y = -100000;
    for(int i = 0; i < this->indices.indices.size(); i++)
    {
        this->top.x = std::max(point_cloud_ptr->at(i).x, this->top.x);
        this->top.y = std::max(point_cloud_ptr->at(i).y, this->top.y);
        this->bottom.x = std::max(point_cloud_ptr->at(i).x, this->bottom.x);
        this->bottom.y = std::max(point_cloud_ptr->at(i).y, this->bottom.y);
    }
}

