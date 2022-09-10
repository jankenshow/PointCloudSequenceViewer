#include <vector>
#include <string>
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

struct BBoxes3D
{
    std::vector<BBox3D> boxes;
};