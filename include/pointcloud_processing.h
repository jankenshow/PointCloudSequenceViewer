#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;

void apply_color (PointCloudT::Ptr cloud);