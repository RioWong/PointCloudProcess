#pragma once
#include "macros.h"
#include "icp.h"
#include <boost/smart_ptr.hpp>

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

class KdTree
{
public:
    KdTree(void);
    ~KdTree(void);

    void setInputCloud(PointCloud<PointXYZRGBA>::ConstPtr cloud);
    int nearestKSearch(const PointXYZRGBA &point, int k, std::vector<int> &k_index, std::vector<double> &k_dis);
private:
    void nearestKSearch(const PointXYZRGBA &point, int k, std::vector<PointXYZRGBA> &k_points);
    boost::shared_ptr<trimesh::KDtree> _kd_tree;
    boost::shared_ptr<trimesh::TriMesh> _mesh;
    std::vector<PointXYZRGBA> _ori_points;
    Eigen::Vector3i _centroid_point;
    double _float_esp;
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END