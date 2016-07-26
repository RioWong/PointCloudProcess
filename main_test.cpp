// pcl_double.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::cout;
using std::endl;
using std::string;
using std::stringstream;

void test_point_cloud()
{
    double pi = 3.141592653589793238;
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 100000; i++) {
        CloudItem p;
        p.x = pi * i;
        p.y = pi * i;
        p.z = pi * i;
        cloud->push_back(p);
    }
    //cout.setf(std::ios::fixed);
    //cout.precision(std::numeric_limits<double>::digits10);
    for (int i = 0; i < 10; i++) {
        //cout << std::setprecision(std::numeric_limits<double>::digits10) << cloud->points[i] << endl;
        cout << cloud->points[i] << endl;
    }
    cout << cloud->size() << endl;
    cout << "-------------------------------" << endl;
}

void test_get_min_max()
{
    CloudItem p_max;
    CloudItem p_min;
    double pi = 3.141592653589793238;
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 100000; i++) {
        CloudItem p;
        p.x = pi * i;
        p.y = pi * i;
        p.z = pi * i;
        cloud->push_back(p);
    }
    cloud_blend_double::PointCloudHelper::getMinMax3D(*cloud, p_min, p_max);
    cout << p_max << endl;
    cout << p_min << endl;
    cout << "-------------------------------" << endl;
}

void test_transform()
{
    double pi = 3.141592653589793238;
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 100000; i++) {
        CloudItem p;
        p.x = pi * i;
        p.y = pi * i;
        p.z = pi * i;
        cloud->push_back(p);
    }

    Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
    rot(0, 3) = 1.0;
    cloud_blend_double::PointCloudHelper::transformPointCloud(*cloud, *cloud, rot);
    for (int i = 0; i < 10; i++) {
        //cout << std::setprecision(std::numeric_limits<double>::digits10) << cloud->points[i] << endl;
        cout << cloud->points[i] << endl;
    }
    cout << "-------------------------------" << endl;
}

void test_remove_nan()
{
    double pi = 3.141592653589793238;
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 10; i++) {
        CloudItem p;
        p.x = std::numeric_limits<double>::signaling_NaN();
        p.y = std::numeric_limits<double>::quiet_NaN();
        p.z = std::numeric_limits<double>::round_error();
        cloud->push_back(p);
    }
    for (int i = 0; i < 10; i++) {
        CloudItem p;
        p.x = pi * i;
        p.y = pi * i;
        p.z = pi * i;
        cloud->push_back(p);
    }
    cout << cloud->size() << endl;
    std::vector<int> index;
    cloud_blend_double::PointCloudHelper::removeNaNFromPointCloud(*cloud, *cloud, index);
    for (int i = 0; i < 10; i++) {
        //cout << std::setprecision(std::numeric_limits<double>::digits10) << cloud->points[i] << endl;
        cout << cloud->points[i] << endl;
    }
    cout << cloud->size() << endl;
    cout << "-------------------------------" << endl;
}

void test_copy_point_cloud()
{
    double pi = 3.141592653589793238;
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 100000; i++) {
        CloudItem p;
        p.x = pi * i;
        p.y = pi * i;
        p.z = pi * i;
        cloud->push_back(p);
    }
    CloudPtr cloud_temp(new Cloud);
    cloud_blend_double::PointCloudHelper::copyPointCloud(*cloud, *cloud_temp);
    for (int i = 0; i < 10; i++) {
        cout << cloud_temp->points[i] << endl;
    }
    cout << "-------------------------------" << endl;
}

void test_voxel_grid()
{
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 10; i++) {
        CloudItem p;
        p.x = i;
        p.y = i;
        p.z = i;
        cloud->push_back(p);
    }
    for (int i = 0; i < 100; i++) {
        CloudItem p;
        p.x = 0.1 * i;
        p.y = 0.1 * i;
        p.z = 0.1 * i;
        cloud->push_back(p);
    }
    cout << cloud->size() << endl;
    cloud_blend_double::VoxelGrid<CloudItem> vox_grid;
    CloudPtr cloud_out(new Cloud);
    vox_grid.setInputCloud(cloud);
    vox_grid.setLeafSize(1, 1, 1);
    vox_grid.filter(*cloud_out);
    cout << cloud_out->size() << endl;
    for (int i = 0; i < 10; i++) {
        cout << cloud_out->points[i] << endl;
    }
    cout << "-------------------------------" << endl;
}

void test_kd_tree()
{
    CloudPtr cloud(new Cloud);
    for (int i = 0; i < 100; i++) {
        CloudItem p;
        p.x = 0.1 * i;
        p.y = 0.1 * i;
        p.z = 0.1 * i;
        cloud->push_back(p);
    }
    //for (int i = 0; i < 100; i++) {
    //    cout << cloud->points[i] << endl;
    //}
    CloudItem point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    std::vector<int> index;
    std::vector<double> dis2;
    KdTreeFLANN<CloudItem> k;
    //cloud_blend_double::KdTree k;
    k.setInputCloud(cloud);
    k.nearestKSearch(point, 10, index, dis2);
    cout << index.size() << endl;
    for (int i = 0; i < index.size(); i++) {
        cout << index[i] << endl;
    }
    cout << endl;
    for (int i = 0; i < dis2.size(); i++) {
        cout << dis2[i] << endl;
    }
    cout << "-------------------------------" << endl;
}

void test_double_io()
{
    cloud_blend_double::PointCloud<cloud_blend_double::PointXYZRGBA>::Ptr cloud(new cloud_blend_double::PointCloud<cloud_blend_double::PointXYZRGBA>);
    for (int i = 0; i < 10; i++) {
        cloud_blend_double::PointXYZRGBA p;
        p.x = M_PI * i;
        p.y = M_PI * i;
        p.z = M_PI * i;
        cloud->push_back(p);
        cout << p << endl;
    }
    cout << endl;
    cloud_blend_double::io::savePCDFileBinary("D:\\1.pcd", *cloud);
    cloud->clear();
    cloud_blend_double::io::loadPCDFile("D:\\1.pcd", *cloud);
    for (int i = 0; i < cloud->size(); i++) {
        cout << cloud->points[i] << endl;
    }
    cout << "----------------------" << endl;
}

void test_float_io()
{
    cloud_blend_double::PointCloud<cloud_blend_double::PointXYZRGBA_F>::Ptr cloud(new cloud_blend_double::PointCloud<cloud_blend_double::PointXYZRGBA_F>);
    for (int i = 0; i < 10; i++) {
        cloud_blend_double::PointXYZRGBA_F p;
        p.x = M_PI * i;
        p.y = M_PI * i;
        p.z = M_PI * i;
        cloud->push_back(p);
        cout << p << endl;
    }
    cout << endl;
    cloud_blend_double::io::savePCDFileBinary("D:\\1.pcd", *cloud);
    cloud->clear();
    cloud_blend_double::io::loadPCDFile("D:\\1.pcd", *cloud);
    for (int i = 0; i < cloud->size(); i++) {
        cout << cloud->points[i] << endl;
    }
    cout << "----------------------" << endl;
}

void cloud_double2float(CloudPtr cloud_in, Cloud_FPtr cloud_out)
{
    for (int i = 0; i < cloud_in->size(); i++) {
        CloudItem_F p;
        p.x = cloud_in->points[i].x;
        p.y = cloud_in->points[i].y;
        p.z = cloud_in->points[i].z;
        cloud_out->push_back(p);
    }
}

void test_icp()
{
    CloudPtr cloud1(new Cloud);
    CloudPtr cloud2(new Cloud);
    string base_dir = "Z:\\linux\\base_pcd_split\\";
    string frame_dir = "Z:\\linux\\frame_pcd_split\\";
    for (int i = 0; i < 20; i++) {
        CloudPtr cloud_temp(new Cloud);
        stringstream ss;
        ss << base_dir << i << ".pcd";
        cloud_blend_double::io::loadPCDFile(ss.str(), *cloud_temp);
        *cloud1 += *cloud_temp;
    }
    for (int i = 0; i < 3; i++) {
        CloudPtr cloud_temp(new Cloud);
        stringstream ss;
        ss << frame_dir << 189 + i << ".pcd";
        cloud_blend_double::io::loadPCDFile(ss.str(), *cloud_temp);
        *cloud2 += *cloud_temp;
    }
    cloud_blend_double::PointCloudHelper::remove_duplicate(cloud1, 0.04);
    cloud_blend_double::PointCloudHelper::remove_duplicate(cloud2, 0.04);
    Cloud_FPtr cloud_o1(new Cloud_F);
    Cloud_FPtr cloud_o2(new Cloud_F);
    cloud_double2float(cloud1, cloud_o1);
    cloud_double2float(cloud2, cloud_o2);
    cloud_blend_double::io::savePCDFile("D:\\1.pcd", *cloud_o1);
    cloud_blend_double::io::savePCDFile("D:\\2.pcd", *cloud_o2);
    Eigen::Matrix4d rot;
    CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudHelper::get_rot_icp(cloud1, cloud2, rot);
    cout << rot << endl;
    cloud_blend_double::PointCloudHelper::transformPointCloud(*cloud2, *cloud2, rot);
    Cloud_FPtr cloud_o3(new Cloud_F);
    Cloud_FPtr cloud_o4(new Cloud_F);
    cloud_double2float(cloud1, cloud_o3);
    cloud_double2float(cloud2, cloud_o4);
    cloud_blend_double::io::savePCDFile("D:\\3.pcd", *cloud_o3);
    cloud_blend_double::io::savePCDFile("D:\\4.pcd", *cloud_o4);
    cout << "----------------------" << endl;
}

int main_test(int argc, char** argv)
{
    //test_point_cloud();

    //test_get_min_max();

    //test_transform();

    //test_remove_nan();

    //test_copy_point_cloud();

    //test_voxel_grid();

    //test_kd_tree();

    //test_double_io();

    //test_float_io();

    test_icp();

    return 0;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END
