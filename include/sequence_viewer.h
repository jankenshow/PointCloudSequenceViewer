#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
// #include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "bbox3d.h"

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;

class SequenceViewer
{
public:
    int pcd_len;
    int current_pcd_id;

    SequenceViewer(std::string pcd_path, bool flag_annot, std::string annot_path);

    void load_pcd_files(const std::string pcd_path);
    void load_point_cloud(const std::string pcd_file_path);
    void update_cloud(int pcd_id);
    void save_pose();

    void showBBox3D(BBox3D &sample);

    int run();

protected:
    bool flag_annot;
    std::string annot_path;
    std::vector<std::string> pcd_files;
    PointCloudT::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void);
void pointPickingEventOccured(const pcl::visualization::PointPickingEvent& event, void *viewer_void);