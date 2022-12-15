#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
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
    std::string cameraparam_save_path;
    int pcd_len;
    int current_pcd_id;

    SequenceViewer(std::string pcd_path, std::string annot_path, std::string cameraparam_path, std::string cameraparam_save_path);

    void load_pcd_files(const std::string pcd_path);
    void load_point_cloud(const std::string pcd_file_path);
    void load_annot_json(std::string pcd_file_path);
    void update_cloud(int pcd_id);
    void load_camerapose(std::string cameraparam_path);
    void save_camerapose();

    void showBBox3D(const BBox3D &bbox);

    int run();

protected:
    std::string annot_path;
    std::vector<std::string> pcd_files;
    PointCloudT::Ptr cloud;
    std::vector<BBox3D> bboxes;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void);
void pointPickingEventOccured(const pcl::visualization::PointPickingEvent& event, void *viewer_void);