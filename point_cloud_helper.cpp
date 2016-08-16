#include "stdafx.h"
#include "point_cloud_helper.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::string;
using std::vector;
using std::min;
using Eigen::Matrix4f;
using Eigen::Matrix4d;
using boost::unordered_map;
using trimesh::TriMesh;
using trimesh::KDtree;
using trimesh::point;
using trimesh::xform;

const bool is_super_cache = true;

int g_cache_icp_num = 0;
uint32_t rgbBlack = static_cast<uint32_t>(0) << 16 | static_cast<uint32_t>(0) << 8 |
    static_cast<uint32_t>(0);
static TriMesh* g_trimesh_cache = NULL;
static KDtree* g_kdtree = NULL;
static int g_cache_num = 0;

//#if _DEBUG
//#pragma comment( lib, "FeatureExtractiond.lib" )
//#else
//#pragma comment( lib, "FeatureExtraction.lib" )
//#endif
//extern "C" void tree_filter(CloudPtr src_cloud, CloudPtr dst_cloud); 
//extern "C" void ground_filter(CloudPtr src_cloud, CloudPtr dst_cloud); 

PointCloudHelper::PointCloudHelper()
{
}


PointCloudHelper::~PointCloudHelper()
{
}

void PointCloudHelper::remove_duplicate(CloudPtr cloud_src, const float voxel_grid_size)
{
    Eigen::Vector4d center;
    PointCloudHelper::compute3DCentroid(*cloud_src, center);

    Matrix4d d_rot = Matrix4d::Identity();
    d_rot.block<3, 1>(0, 3) = -center.block<3, 1>(0, 0);

    CloudPtr cloud_temp2(new Cloud);
    copyPointCloud(*cloud_src, *cloud_temp2);

    PointCloudHelper::transformPointCloud(*cloud_temp2, *cloud_temp2, d_rot);

    VoxelGrid<CloudItem> vox_grid;
    vox_grid.setInputCloud(cloud_temp2);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*cloud_src);

    d_rot.block<3, 1>(0, 3) = center.block<3, 1>(0, 0);
    PointCloudHelper::transformPointCloud(*cloud_src, *cloud_src, d_rot);
    //pcl::copyPointCloud(*cloudTemp2, *cloud_src);
}

void PointCloudHelper::remove_duplicate(CloudPtr cloud_src, const float voxel_grid_size, int min_num)
{
    CloudPtr cloud_temp2(new Cloud);
    PointCloudHelper::copyPointCloud(*cloud_src, *cloud_temp2);
    PointCloudHelper::remove_duplicate(cloud_temp2, voxel_grid_size);
    if (cloud_temp2->size() >= min_num) {
        copyPointCloud(*cloud_temp2, *cloud_src);
    }
}

float PointCloudHelper::get_rot_icp(CloudPtr cloud_src, CloudPtr cloud_temp,
                                    Matrix4d& mat_rot, bool do_scale, bool do_affine)
{
    CloudPtr cloud_all(new Cloud);
    *cloud_all += *cloud_src;
    *cloud_all += *cloud_temp;
    Eigen::Vector4d centroid;
    PointCloudHelper::compute3DCentroid(*cloud_all, centroid);
    cloud_all.reset();

    int verbose = 0;
    bool bulkmode = false;

    TriMesh::set_verbose(verbose);
    TriMesh* mesh1 = new TriMesh();
    TriMesh* mesh2 = new TriMesh();
    mesh1->vertices.resize(cloud_src->size());
    mesh2->vertices.resize(cloud_temp->size());
    for (int i = 0; i < cloud_src->size(); i++) {
        mesh1->vertices[i] = point(
            cloud_src->points[i].x - centroid(0), 
            cloud_src->points[i].y - centroid(1),
            cloud_src->points[i].z - centroid(2));
    }
    for (int i = 0; i < cloud_temp->size(); i++) {
        mesh2->vertices[i] = point(
            cloud_temp->points[i].x - centroid(0), 
            cloud_temp->points[i].y - centroid(1),
            cloud_temp->points[i].z - centroid(2));
    }


    xform xf1;
    xform xf2;

    KDtree* kd1 = new KDtree(mesh1->vertices);
    KDtree* kd2 = new KDtree(mesh2->vertices);
    vector<float> weights1, weights2;

    if (bulkmode) {
        float area1 = mesh1->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
        float area2 = mesh2->stat(TriMesh::STAT_TOTAL, TriMesh::STAT_FACEAREA);
        float overlap_area, overlap_dist;
        find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2, overlap_area, overlap_dist);
        float frac_overlap = overlap_area / min(area1, area2);
        if (frac_overlap < 0.1f) {
            TriMesh::eprintf("Insufficient overlap\n");
            exit(1);
        } else {
            TriMesh::dprintf("%.1f%% overlap\n", frac_overlap * 100.0);
        }
    }
    float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2, 0, verbose,
                    do_scale, do_affine);
    //if (err >= 0.0f)
    //  err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
    //  verbose, do_scale, do_affine);

    if (err < 0.0f) {
        TriMesh::eprintf("ICP failed\n");
        //exit(1);
    }

    //TriMesh::eprintf("ICP succeeded - distance = %f\n", err);
    delete kd1;
    delete kd2;
    delete mesh1;
    delete mesh2;

    if (bulkmode) {
        xform xf12 = inv(xf2) * xf1;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat_rot(i, j) = xf12[i + 4 * j];
            }
        }
    } else {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mat_rot(i, j) = xf2[i + 4 * j];
            }
        }
    }
    // base_cloud           = rot' *  frame_cloud
    // base_cloud           = R'   *  frame_cloud + T'
    // base_cloud - delta_t = rot  * (frame_cloud - delta_t)
    // base_cloud - delta_t = R    * (frame_cloud - delta_t) + T
    // base_cloud           = R    *  frame_cloud - R * delta_t + delta_t + T
    // therefore, R' = R, T' = T - R * delta_t + delta_t
    mat_rot.block<3, 1>(0, 3) = mat_rot.block<3, 1>(0, 3) - mat_rot.block<3, 3>(0, 0) * centroid.block<3, 1>(0, 0) + centroid.block<3, 1>(0, 0);
    return err;
}

//float PointCloudHelper::get_rot_icp_filter(CloudPtr cloud_src, CloudPtr cloud_temp, Eigen::Matrix4f& mat_rot, bool do_scale, bool do_affine)
//{
//    CloudPtr cloud_src_filtered(new Cloud());
//	CloudPtr cloud_dst_filtered(new Cloud());
//	PointCloudHelper::point_cloud_feature_filter(cloud_src, cloud_src_filtered);
//	PointCloudHelper::point_cloud_feature_filter(cloud_temp, cloud_dst_filtered);
//	//PointCloudHelper::point_cloud_feature_filter(cloud_temp, cloud_dst_filtered);
//    return get_rot_icp(cloud_src_filtered, cloud_dst_filtered, mat_rot, do_scale, do_affine);
//}

void PointCloudHelper::change_cloud_rgb(CloudPtr cloud_src, int r, int g, int b)
{
    uint32_t rgbBlack = static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 |
        static_cast<uint32_t>(b);

    for (int i = 0; i < cloud_src->size(); i ++) {
        cloud_src->points[i].rgba = rgbBlack;
    }
}

//void PointCloudHelper::point_cloud_feature_filter(CloudPtr cloud_src, CloudPtr cloud_dst)
//{
//    ground_filter(cloud_src, cloud_dst);
//}

double PointCloudHelper::point_dis2(const PointXYZRGBA& p1, const PointXYZRGBA& p2)
{
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
}


void PointCloudHelper::cloud_double2float(CloudPtr cloud_in, Cloud_FPtr cloud_out, PointXYZRGBA center)
{
    for (int i = 0; i < cloud_in->size(); i++) {
        CloudItem_F p;
        p.x = static_cast<float>(cloud_in->points[i].x - center.x);
        p.y = static_cast<float>(cloud_in->points[i].y - center.y);
        p.z = static_cast<float>(cloud_in->points[i].z - center.z);
        cloud_out->push_back(p);
    }
}

void PointCloudHelper::cloud_double2float(CloudPtr cloud_in, Cloud_FPtr cloud_out)
{
    for (int i = 0; i < cloud_in->size(); i++) {
        CloudItem_F p;
        p.x = std::fmod(cloud_in->points[i].x,10000);
        p.y = std::fmod(cloud_in->points[i].y,10000);
        p.z = std::fmod(cloud_in->points[i].z,10000);
        cloud_out->push_back(p);
    }
}

void PointCloudHelper::dpcd2fpcd(string str_path)
{
    namespace bs = boost::filesystem;
    string path_pcd = str_path;
    bs::path path_path_pcd(path_pcd);
    std::vector<string> paths;
    if (bs::is_directory(path_pcd)) {
        std::vector<string> paths_temp;
        FileHelper::get_dir_files(path_pcd, paths_temp, ".pcd");
        for (int i = 0; i < paths_temp.size(); i++) {
            if (paths_temp[i].find("_f.pcd") == std::string::npos) {
                paths.push_back(paths_temp[i]);
            }
        }
    } else {
        paths.push_back(path_pcd);
    }
    for (int i = 0; i < paths.size(); i++) {
        std::cout << paths[i] << std::endl;
        if (paths[i].find("_f.pcd") != std::string::npos) {
            continue;
        }
        CloudPtr cloud_temp(new Cloud);
        io::loadPCDFile(paths[i], *cloud_temp);
        Cloud_FPtr cloud_o2(new Cloud_F);
        PointCloudHelper::cloud_double2float(cloud_temp, cloud_o2);
        io::savePCDFile(paths[i] + "_f.pcd", *cloud_o2);
    }
}

void PointCloudHelper::remove_center_points(CloudPtr cloud, CloudPtr cloud_out, const Eigen::Matrix4d& rot, const double dis)
{
    const double dis2 = dis * dis;
    bool compute_center = true;
    Eigen::Vector4d center;
    CloudItem center_point;
    if (compute_center) {
        compute3DCentroid(*cloud, center);
    } else {
        center = rot.block<4, 1>(0, 3);
    }

    center_point.x = center[0];
    center_point.y = center[1];
    center_point.z = center[2];

    CloudPtr cloud_temp_out(new Cloud);
    for (int i = 0; i < cloud->size(); ++i) {
        const CloudItem& point = cloud->points[i];
        if (std::abs(point.x - center[0]) > dis || std::abs(point.y - center[1]) > dis ||
            std::abs(point.z - center[2]) > dis) {
            cloud_temp_out->push_back(point);
            continue;
        }
        if (point_dis2(point, center_point) >  dis2) {
            cloud_temp_out->push_back(point);
        }
    }
    *cloud_out = *cloud_temp_out;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END