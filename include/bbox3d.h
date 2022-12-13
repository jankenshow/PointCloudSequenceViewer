#pragma once

#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
// #include <boost/optional.hpp>
#include <Eigen/Dense>


struct BBox3D
{
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    double width;
    double height;
    double depth;
    std::string id;
};


inline bool load_annot(std::string annot_file, std::vector<BBox3D> &bboxes)
{
    namespace bpt = boost::property_tree;

    bool ret = true;
    bpt::ptree pt;
    bpt:read_json(annot_file, pt);

    try {
        bpt::ptree lidar_0 = pt.get_child("Lidar").begin()->second;
        Eigen::Vector3f lidar_0_trans(
            lidar_0.get<float>("Location.x"), lidar_0.get<float>("Location.y"), lidar_0.get<float>("Location.z")
        );
        Eigen::Quaternionf lidar_0_quat(
            lidar_0.get<float>("Rotation.w"), lidar_0.get<float>("Rotation.x"), lidar_0.get<float>("Rotation.y"), lidar_0.get<float>("Rotation.z")
        );

        int count = 0;
        BOOST_FOREACH (const bpt::ptree::value_type& child, pt.get_child("BoundingBox3D")) 
        {
            const bpt::ptree& info = child.second;
            Eigen::Vector3f bbox_center(
                info.get<float>("Origin.x"), info.get<float>("Origin.y"), info.get<float>("Origin.z")
            );
            Eigen::Quaternionf bbox_rot(
                info.get<float>("Rotation.w"), info.get<float>("Rotation.x"), info.get<float>("Rotation.y"), info.get<float>("Rotation.z")
            );
            Eigen::Vector3f bbox_center_lidar_coord = lidar_0_quat.inverse() * (bbox_center - lidar_0_trans);
            Eigen::Quaternionf bbox_rot_lidar_coord = lidar_0_quat.inverse() * bbox_rot;

            float width, height, depth;
            width = info.get<float>("Extent.x");
            height = info.get<float>("Extent.y");
            depth = info.get<float>("Extent.z");

            std::string id = info.get<std::string>("Label") + "_";
            id += std::to_string(count);

            bboxes.push_back(BBox3D{bbox_center_lidar_coord, bbox_rot_lidar_coord, width, height, depth, id});
            ++count;
        }
    } catch (bpt::json_parser_error &e) {
        ret = false;
        std::cout << e.what() << std::endl;
    }

    return ret;
}