#include "StdAfx.h"
#include "kd_tree.h"
#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "ICP.h"


CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

using std::vector;

using trimesh::TriMesh;
using trimesh::KDtree;
using trimesh::point;
typedef boost::shared_ptr<TriMesh> TriMeshPtr;
typedef boost::shared_ptr<KDtree> KDtreePtr;

KdTree::KdTree(void)
    :_centroid_point(Eigen::Vector3i::Zero()),
    _float_esp(static_cast<double>(std::numeric_limits<float>::epsilon()))
{
}


KdTree::~KdTree(void)
{
}

void KdTree::setInputCloud(PointCloud<PointXYZRGBA>::ConstPtr cloud)
{
    _mesh.reset(new TriMesh);
    _mesh->vertices.resize(cloud->size());
    Eigen::Vector4d centroid;
    CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudHelper::compute3DCentroid(*cloud, centroid);
    _centroid_point(0) = centroid(0);
    _centroid_point(1) = centroid(1);
    _centroid_point(2) = centroid(2);
    //std::cout << _centroid_point << std::endl << std::endl;
    for (int i = 0; i < cloud->size(); i++) {
        _mesh->vertices[i] = point(static_cast<float>(cloud->points[i].x - _centroid_point(0)), 
            static_cast<float>(cloud->points[i].y - _centroid_point(1)),
            static_cast<float>(cloud->points[i].z - _centroid_point(2)));
    }

    _ori_points.clear();
    _ori_points.resize(cloud->size());
    for (int i = 0; i < cloud->size(); i++) {
        _ori_points[i].x = cloud->points[i].x;
        _ori_points[i].y = cloud->points[i].y;
        _ori_points[i].z = cloud->points[i].z;
    }
    //for (int i = 0; i < cloud->size(); i++) {
    //    std::cout << _mesh->vertices[i][0] << " " << _mesh->vertices[i][1] << " " << _mesh->vertices[i][2] << std::endl;
    //}
    //std::cout << "-------------------------------" << std::endl;

    _kd_tree.reset(new KDtree(_mesh->vertices));
}
void KdTree::nearestKSearch(const PointXYZRGBA &point, int k, vector<PointXYZRGBA> &k_points)
{
    k_points.clear();

    trimesh::point p(
        static_cast<double>(point.x - _centroid_point(0)), 
        static_cast<double>(point.y - _centroid_point(1)), 
        static_cast<double>(point.z - _centroid_point(2)));
    vector<const float *> points;
    _kd_tree->find_k_closest_to_pt(points, k, p);
    for (int i = 0; i < points.size(); i++) {
        PointXYZRGBA point_temp;
        point_temp.x = static_cast<double>(points[i][0] + _centroid_point(0));
        point_temp.y = static_cast<double>(points[i][1] + _centroid_point(1));
        point_temp.z = static_cast<double>(points[i][2] + _centroid_point(2));
        k_points.push_back(point_temp);
    }
    //std::cout << "-------------------------------" << std::endl;
}
int KdTree::nearestKSearch(const PointXYZRGBA &point, int k, vector<int> &k_index, vector<double> &k_dis2)
{
    k_index.clear();
    k_dis2.clear();

    vector<PointXYZRGBA> k_points;
    nearestKSearch(point, k, k_points);

    k_index.resize(k_points.size());
    k_dis2.resize(k_points.size());

    for (int i = 0; i < k_points.size(); i++) {
        int index = -1;
        for (int j = 0; j < _ori_points.size(); j++) {
// 			const double eps = 0.01;
// 			if (std::abs(k_points[i].x - _ori_points[j].x) > eps ||
// 				std::abs(k_points[i].y - _ori_points[j].y) > eps ||
// 				std::abs(k_points[i].z - _ori_points[j].z) > eps) {
// 					continue;
// 			}

            double dis2 = PointCloudHelper::point_dis2(k_points[i], _ori_points[j]);
			k_dis2[i]=dis2;
            if (dis2 <= _float_esp) {
                index = j;
                break;
            }
        }
        if (index < 0) {
            k_index[i] = -1;
        } else {
            k_index[i] = index;
        }
    }

//     for (int i = 0; i < k_dis2.size(); i++) {
//         k_dis2[i] = PointCloudHelper::point_dis2(point, _ori_points[k_index[i]]);
//     }
    return k_points.size();
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END