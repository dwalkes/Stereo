#include "segment.h"
#include <algorithm> 

Segment::Segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointIndices ind)
{
    this->indices = ind;
    this->top.x = 100000;
    this->top.y = 100000;
    this->bottom.x = -100000;
    this->bottom.y = -100000;
    this->mean = 0;
    for(int j = 0; j < this->indices.indices.size(); j++)
    {
        int i = this->indices.indices[j];
        this->top.x = std::min(point_cloud_ptr->at(i).x, this->top.x);
        this->top.y = std::min(point_cloud_ptr->at(i).y, this->top.y);
        this->bottom.x = std::max(point_cloud_ptr->at(i).x, this->bottom.x);
        this->bottom.y = std::max(point_cloud_ptr->at(i).y, this->bottom.y);
        this->mean += point_cloud_ptr->at(i).z;
    }
    this->mean /= this->indices.indices.size();
    this->width = this->bottom.x - this->top.x;
    this->height = this->bottom.y - this->top.y;
    this->center.x = this->top.x + this->width/2;
    this->center.y = this->top.y + this->height/2;
}

