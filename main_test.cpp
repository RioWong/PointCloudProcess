// pcl_double.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <boost/timer/timer.hpp>

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using std::vector;
using std::pair;
using boost::unordered_map;

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

void test() {

//        // base
//        CloudGrid::instance().clear();
//
//#if 0
//        int base_cache_num = 20;
//        vector<CloudPtr> base_clouds(base_end_stamp - base_start_stamp + 1 + 2 * base_cache_num);
//#pragma omp parallel for
//        for (int index = base_start_stamp - base_cache_num; index <= base_end_stamp + base_cache_num; index++) {
//            int i = index - base_start_stamp + base_cache_num;
//            base_clouds[i] = boost::make_shared<Cloud>();
//            if (i < rots[0]._stamp || i > rots[rots.size() - 1]._stamp) {
//                cout << "stamp invalid, ignore " << i << endl;
//                continue;
//            }
//            stringstream ss;
//            ss << i << ".pcd";
//            string pcd_file_path = (pcd_dir_path / ss.str()).string();
//            cout << "load pcd file " << pcd_file_path << endl;
//            Cloud_FPtr cloud_temp(new Cloud_F);
//            io::loadPCDFile(pcd_file_path, *cloud_temp);
//            base_clouds[i]->resize(cloud_temp->size());
//            for (int j = 0; j < cloud_temp->size(); j++) {
//                base_clouds[i]->points[j].x = cloud_temp->points[j].x;
//                base_clouds[i]->points[j].y = cloud_temp->points[j].y;
//                base_clouds[i]->points[j].z = cloud_temp->points[j].z;
//                base_clouds[i]->points[j].r = cloud_temp->points[j].r;
//                base_clouds[i]->points[j].g = cloud_temp->points[j].g;
//                base_clouds[i]->points[j].b = cloud_temp->points[j].b;
//                base_clouds[i]->points[j].a = cloud_temp->points[j].a;
//            }
//            if (stamp_rot_map.find(i) == stamp_rot_map.end()) {
//                cout << "base stamp cannot find, ignore " << i << endl;
//                continue;
//            }
//            PointCloudHelper::remove_duplicate(base_clouds[i], 0.04);
//            PointCloudHelper::transformPointCloud(*base_clouds[i], *base_clouds[i], stamp_rot_map[i]);
//        }
//
//        CloudPtr base_cloud_overall(new Cloud);
//        //for (int i = 0; i < base_clouds.size(); i++) {
//        //    *base_cloud_overall += *base_clouds[i];
//        //}
//        int segment_count = 40;
//        for (int i = 0; i < base_clouds.size() / segment_count + 1; i++) {
//            CloudPtr rpca_cloud(new Cloud);
//            for (int j = i * segment_count; j < i * segment_count + segment_count; j++) {
//                if (j >= base_clouds.size()) {
//                    break;
//                }
//                *rpca_cloud += *base_clouds[j];
//            }
//            CloudPtr cloud_temp(new Cloud);
//            cout << "processing rpca...[" << i * segment_count << "," << i * segment_count + segment_count - 1 << "]";
//            tree_filter(rpca_cloud, cloud_temp);
//            cout << "done." << endl;
//            *base_cloud_overall += *cloud_temp;
//        }
//        cout << "saving D:\\rpca.pcd" << endl;
//        io::savePCDFileBinary("D:\\rpca.pcd", *base_cloud_overall);
//#else
//        CloudPtr base_cloud_overall(new Cloud);
//        cout << "load from file" << endl;
//        io::loadPCDFile("Z:\\20160719\\rpca.pcd", *base_cloud_overall);
//#endif
//        //CloudPtr rpca_cloud(new Cloud);
//        //tree_filter(base_cloud_overall, rpca_cloud);
//        cout << "add cloud grid..." << endl;
//        CloudGrid::instance().add_cloud_internal(base_cloud_overall);
//
//        // frame
//
//        // 进行5m分组
//        vector<vector<uint64_t> > seg_group;
//        Eigen::Matrix4d start_rot = Eigen::Matrix4d::Identity();
//        int frame_index = frame_start_stamp;
//        for (frame_index = frame_start_stamp; frame_index < frame_end_stamp; frame_index++) {
//            uint64_t stamp = frame_index;
//            if (stamp_rot_map.find(stamp) == stamp_rot_map.end()) {
//                cout << "frame stamp cannot find, ignore " << frame_index << endl;
//                continue;
//            }
//            start_rot = stamp_rot_map[stamp];
//            vector<uint64_t> seg_temp;
//            seg_temp.push_back(stamp);
//            seg_group.push_back(seg_temp);
//            frame_index++;
//            break;
//        }
//        if (seg_group.size() == 0) {
//            cout << "seg_group.size() == 0, return" << endl;
//            return;
//        }
//        for (; frame_index <= frame_end_stamp; frame_index++) {
//            uint64_t stamp = frame_index;
//            if (stamp_rot_map.find(stamp) == stamp_rot_map.end()) {
//                cout << "frame stamp cannot find, ignore " << frame_index << endl;
//                continue;
//            }
//            vector<uint64_t>& seg_temp = seg_group[seg_group.size() - 1];
//            double dis2 = distance_sqr_rot(stamp_rot_map[stamp], start_rot);
//            if (dis2 >= 25.0) {
//                start_rot = stamp_rot_map[stamp];
//                vector<uint64_t> seg_temp2;
//                seg_temp2.push_back(stamp);
//                seg_group.push_back(seg_temp2);
//            } else {
//                seg_temp.push_back(stamp);
//            }
//        }
//
//        for (int i = 0; i < seg_group.size(); i++) {
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                cout << seg_group[i][j] << ",";
//            }
//            cout << endl;
//        }
//
//        vector<CloudPtr> frame_merged_cloud_vec(seg_group.size());
//#if 1
//#pragma omp parallel for
//        for (int i = 0; i < seg_group.size(); i++) {
//
//            frame_merged_cloud_vec[i] = boost::make_shared<Cloud>();
//
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                uint64_t cur_stamp = seg_group[i][j];
//                stringstream ss;
//                ss << cur_stamp << ".pcd";
//                string pcd_file_path = (pcd_dir_path / ss.str()).string();
//                cout << "load pcd file " << pcd_file_path << endl;
//                Cloud_FPtr cloud_temp(new Cloud_F);
//                CloudPtr cloud_temp2(new Cloud);
//                io::loadPCDFile(pcd_file_path, *cloud_temp);
//                vector<int> nan_index;
//                PointCloudHelper::removeNaNFromPointCloud(*cloud_temp, *cloud_temp, nan_index);
//                cloud_temp2->resize(cloud_temp->size());
//                for (int k = 0; k < cloud_temp->size(); k++) {
//                    cloud_temp2->points[k].x = cloud_temp->points[k].x;
//                    cloud_temp2->points[k].y = cloud_temp->points[k].y;
//                    cloud_temp2->points[k].z = cloud_temp->points[k].z;
//                    cloud_temp2->points[k].r = cloud_temp->points[k].r;
//                    cloud_temp2->points[k].g = cloud_temp->points[k].g;
//                    cloud_temp2->points[k].b = cloud_temp->points[k].b;
//                    cloud_temp2->points[k].a = cloud_temp->points[k].a;
//                }
//                if (stamp_rot_map.find(cur_stamp) == stamp_rot_map.end()) {
//                    cout << "frame stamp cannot find, ignore " << cur_stamp << endl;
//                    continue;
//                }
//                PointCloudHelper::remove_duplicate(cloud_temp2, 0.04);
//                PointCloudHelper::transformPointCloud(*cloud_temp2, *cloud_temp2, stamp_rot_map[cur_stamp]);
//
//                *frame_merged_cloud_vec[i] += *cloud_temp2;
//            }
//        }
//        //for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//        //    stringstream ss;
//        //    ss << i << ".pcd";
//        //    cout << "save file " << ss.str() << endl;
//        //    if (frame_merged_cloud_vec[i]->size() == 0) {
//        //        cout << "point cloud empty,skip" << endl;
//        //        continue;
//        //    }
//        //    io::savePCDFileBinary("D:\\"+ ss.str(), *frame_merged_cloud_vec[i]);
//        //}
//
//        int segment_count2 = 10;
//        for (int i = 0; i < frame_merged_cloud_vec.size() / segment_count2 + 1; i++) {
//            CloudPtr rpca_cloud(new Cloud);
//            int stamp_count = 0;
//            for (int j = i * segment_count2; j < i * segment_count2 + segment_count2; j++, stamp_count++) {
//                if (j >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                int before_length = rpca_cloud->size();
//                *rpca_cloud += *frame_merged_cloud_vec[j];
//                int after_length = rpca_cloud->size();
//                for (int index = before_length; index < after_length; index++) {
//                    rpca_cloud->points[index].stamp_id = stamp_count;
//                }
//            }
//            CloudPtr cloud_temp(new Cloud);
//            cout << "processing rpca...[" << i * segment_count2 << "," << i * segment_count2 + segment_count2 - 1 << "]";
//            tree_filter(rpca_cloud, cloud_temp);
//            //cloud_temp = rpca_cloud;
//            cout << "done." << endl;
//
//            vector<CloudPtr> cloud_temp_vec(segment_count2);
//            for (int j = 0; j < cloud_temp_vec.size(); j++) {
//                cloud_temp_vec[j] = boost::make_shared<Cloud>();
//            }
//            for (int j = 0; j < cloud_temp->size(); j++) {
//                int index = cloud_temp->points[j].stamp_id;
//                if (index >= 0 && index < segment_count2) {
//                    cloud_temp_vec[index]->points.push_back(cloud_temp->points[j]);
//                }
//            }
//            for (int j = 0; j < cloud_temp_vec.size(); j++) {
//                int index = i * segment_count2 + j;
//                if (index >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                *frame_merged_cloud_vec[index] = *cloud_temp_vec[j];
//            }
//            cloud_temp_vec.clear();
//        }
//#else
//#pragma omp parallel for
//        for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//            frame_merged_cloud_vec[i] = boost::make_shared<Cloud>();
//            stringstream ss;
//            ss << "Z:\\20160719\\frame_pca\\" << i << ".pcd";
//            cout << "load pcd " << ss.str() << endl;
//            io::loadPCDFile(ss.str(), *frame_merged_cloud_vec[i]);
//        }
//#endif
//
//        //for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//        //    stringstream ss;
//        //    ss << i << ".pcd";
//        //    cout << "save file " << ss.str() << endl;
//        //    if (frame_merged_cloud_vec[i]->size() == 0) {
//        //        cout << "point cloud empty,skip" << endl;
//        //        continue;
//        //    }
//        //    io::savePCDFileBinary("D:\\"+ ss.str(), *frame_merged_cloud_vec[i]);
//        //}
//
//        int icp_cache_num = 10;
//        float icp_threshold = 0.13;
//        vector<Eigen::Matrix4d> rot_vec(frame_merged_cloud_vec.size());
//        for (int i = 0; i < rot_vec.size(); i++) {
//            rot_vec[i] = Eigen::Matrix4d::Zero();
//        }
//        for (int i = 0; i < frame_merged_cloud_vec.size() - icp_cache_num; i++) {
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                cout << seg_group[i][j];
//                if (j < seg_group[i].size() - 1) {
//                    cout << ",";
//                }
//            }
//            cout << endl;
//            CloudPtr cloud(new Cloud);
//            CloudPtr cloud_cache(new Cloud);
//            for (int cache_index = 0; cache_index < icp_cache_num; cache_index++) {
//                int cur_index = i + cache_index;
//                if (cur_index >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                *cloud += *frame_merged_cloud_vec[cur_index];
//            }
//            PointCloudHelper::remove_duplicate(cloud, 0.04);
//            CloudItem min_xyz;
//            CloudItem max_xyz;
//            PointCloudHelper::getMinMax3D(*cloud, min_xyz, max_xyz);
//            float d_length = 30;
//            min_xyz.x -= d_length;
//            min_xyz.y -= d_length;
//            min_xyz.z -= d_length;
//            max_xyz.x += d_length;
//            max_xyz.y += d_length;
//            max_xyz.z += d_length;
//            CloudGrid::instance().get_cloud_with_pos(cloud_cache,
//                min_xyz, max_xyz);
//            Eigen::Matrix4d pair_transform = Eigen::Matrix4d::Identity();
//            cout << "before icp cloud1 " << cloud_cache->size() << " cloud2 " << cloud->size() << endl;
//
//            size_t cloud_point_threshold = 10000;
//            if (cloud_cache->size() < cloud_point_threshold || cloud->size() < cloud_point_threshold
//                || cloud->size() / (float)cloud_cache->size() >= 10) {
//                    cout << "reg: size unvalid identity stamp " << cloud->header.stamp << endl;
//                    //process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Identity(), cur_index, -2.0, icp_threshold);
//                    continue;
//            }
//
//            //float icp_dis = PointCloudHelper::get_rot_icp_filter(cloud_cache, cloud_tras, pair_transform, true, false);
//            float icp_dis = PointCloudHelper::get_rot_icp(cloud_cache, cloud, pair_transform, false, false);
//            cout << "reg: icp ret: " << icp_dis << endl;
//            cout << "reg: icp rot: " << endl << pair_transform << endl;
//            if (icp_dis < icp_threshold) {
//                rot_vec[i] = pair_transform;
//            }
//        }
//
//        bool is_failed = true;
//        Eigen::Matrix4d rot_first;
//        Eigen::Matrix4d rot_last;
//        int index_first = -1;
//        int index_last = -1;
//        for (int i = 0; i < rot_vec.size(); i++) {
//            if (rot_vec[i] != Eigen::Matrix4d::Zero()) {
//                is_failed = false;
//                rot_first = rot_vec[i];
//                index_first = i;
//                break;
//            }
//        }
//        if (is_failed) {
//            cout << "process failed" << endl;
//            return;
//        }
//
//        for (int i = rot_vec.size() - 1; i >= 0; i--) {
//            if (rot_vec[i] != Eigen::Matrix4d::Zero()) {
//                is_failed = false;
//                rot_last = rot_vec[i];
//                index_last = i;
//                break;
//            }
//        }
//
//        // 向前填充
//        if (index_first != 0) {
//            for (int i = 0; i < index_first; i++) {
//                rot_vec[i] = rot_first;
//            }
//        }
//        // 向后填充
//        for (int i = index_last + 1; i < rot_vec.size(); i++) {
//            rot_vec[i] = rot_last;
//        }
//        // 中间填充
//        for (int i = index_first; i < index_last; i++) {
//            if (rot_vec[i] == Eigen::Matrix4d::Zero()) {
//                rot_vec[i] = rot_vec[i - 1];
//            }
//        }
//
//        for (int i = 0; i < rot_vec.size(); i++) {
//            const Eigen::Matrix4d& rot = rot_vec[i];
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                uint64_t cur_stamp = seg_group[i][j];
//                stamp_rot_map[cur_stamp] = rot * stamp_rot_map[cur_stamp];
//            }
//        }
//        // base
//        CloudGrid::instance().clear();
//
//#if 0
//        int base_cache_num = 20;
//        vector<CloudPtr> base_clouds(base_end_stamp - base_start_stamp + 1 + 2 * base_cache_num);
//#pragma omp parallel for
//        for (int index = base_start_stamp - base_cache_num; index <= base_end_stamp + base_cache_num; index++) {
//            int i = index - base_start_stamp + base_cache_num;
//            base_clouds[i] = boost::make_shared<Cloud>();
//            if (i < rots[0]._stamp || i > rots[rots.size() - 1]._stamp) {
//                cout << "stamp invalid, ignore " << i << endl;
//                continue;
//            }
//            stringstream ss;
//            ss << i << ".pcd";
//            string pcd_file_path = (pcd_dir_path / ss.str()).string();
//            cout << "load pcd file " << pcd_file_path << endl;
//            Cloud_FPtr cloud_temp(new Cloud_F);
//            io::loadPCDFile(pcd_file_path, *cloud_temp);
//            base_clouds[i]->resize(cloud_temp->size());
//            for (int j = 0; j < cloud_temp->size(); j++) {
//                base_clouds[i]->points[j].x = cloud_temp->points[j].x;
//                base_clouds[i]->points[j].y = cloud_temp->points[j].y;
//                base_clouds[i]->points[j].z = cloud_temp->points[j].z;
//                base_clouds[i]->points[j].r = cloud_temp->points[j].r;
//                base_clouds[i]->points[j].g = cloud_temp->points[j].g;
//                base_clouds[i]->points[j].b = cloud_temp->points[j].b;
//                base_clouds[i]->points[j].a = cloud_temp->points[j].a;
//            }
//            if (stamp_rot_map.find(i) == stamp_rot_map.end()) {
//                cout << "base stamp cannot find, ignore " << i << endl;
//                continue;
//            }
//            PointCloudHelper::remove_duplicate(base_clouds[i], 0.04);
//            PointCloudHelper::transformPointCloud(*base_clouds[i], *base_clouds[i], stamp_rot_map[i]);
//        }
//
//        CloudPtr base_cloud_overall(new Cloud);
//        //for (int i = 0; i < base_clouds.size(); i++) {
//        //    *base_cloud_overall += *base_clouds[i];
//        //}
//        int segment_count = 40;
//        for (int i = 0; i < base_clouds.size() / segment_count + 1; i++) {
//            CloudPtr rpca_cloud(new Cloud);
//            for (int j = i * segment_count; j < i * segment_count + segment_count; j++) {
//                if (j >= base_clouds.size()) {
//                    break;
//                }
//                *rpca_cloud += *base_clouds[j];
//            }
//            CloudPtr cloud_temp(new Cloud);
//            cout << "processing rpca...[" << i * segment_count << "," << i * segment_count + segment_count - 1 << "]";
//            tree_filter(rpca_cloud, cloud_temp);
//            cout << "done." << endl;
//            *base_cloud_overall += *cloud_temp;
//        }
//        cout << "saving D:\\rpca.pcd" << endl;
//        io::savePCDFileBinary("D:\\rpca.pcd", *base_cloud_overall);
//#else
//        CloudPtr base_cloud_overall(new Cloud);
//        cout << "load from file" << endl;
//        io::loadPCDFile("Z:\\20160719\\rpca.pcd", *base_cloud_overall);
//#endif
//        //CloudPtr rpca_cloud(new Cloud);
//        //tree_filter(base_cloud_overall, rpca_cloud);
//        cout << "add cloud grid..." << endl;
//        CloudGrid::instance().add_cloud_internal(base_cloud_overall);
//
//        // frame
//
//        // 进行5m分组
//        vector<vector<uint64_t> > seg_group;
//        Eigen::Matrix4d start_rot = Eigen::Matrix4d::Identity();
//        int frame_index = frame_start_stamp;
//        for (frame_index = frame_start_stamp; frame_index < frame_end_stamp; frame_index++) {
//            uint64_t stamp = frame_index;
//            if (stamp_rot_map.find(stamp) == stamp_rot_map.end()) {
//                cout << "frame stamp cannot find, ignore " << frame_index << endl;
//                continue;
//            }
//            start_rot = stamp_rot_map[stamp];
//            vector<uint64_t> seg_temp;
//            seg_temp.push_back(stamp);
//            seg_group.push_back(seg_temp);
//            frame_index++;
//            break;
//        }
//        if (seg_group.size() == 0) {
//            cout << "seg_group.size() == 0, return" << endl;
//            return;
//        }
//        for (; frame_index <= frame_end_stamp; frame_index++) {
//            uint64_t stamp = frame_index;
//            if (stamp_rot_map.find(stamp) == stamp_rot_map.end()) {
//                cout << "frame stamp cannot find, ignore " << frame_index << endl;
//                continue;
//            }
//            vector<uint64_t>& seg_temp = seg_group[seg_group.size() - 1];
//            double dis2 = distance_sqr_rot(stamp_rot_map[stamp], start_rot);
//            if (dis2 >= 25.0) {
//                start_rot = stamp_rot_map[stamp];
//                vector<uint64_t> seg_temp2;
//                seg_temp2.push_back(stamp);
//                seg_group.push_back(seg_temp2);
//            } else {
//                seg_temp.push_back(stamp);
//            }
//        }
//
//        for (int i = 0; i < seg_group.size(); i++) {
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                cout << seg_group[i][j] << ",";
//            }
//            cout << endl;
//        }
//
//        vector<CloudPtr> frame_merged_cloud_vec(seg_group.size());
//#if 1
//#pragma omp parallel for
//        for (int i = 0; i < seg_group.size(); i++) {
//
//            frame_merged_cloud_vec[i] = boost::make_shared<Cloud>();
//
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                uint64_t cur_stamp = seg_group[i][j];
//                stringstream ss;
//                ss << cur_stamp << ".pcd";
//                string pcd_file_path = (pcd_dir_path / ss.str()).string();
//                cout << "load pcd file " << pcd_file_path << endl;
//                Cloud_FPtr cloud_temp(new Cloud_F);
//                CloudPtr cloud_temp2(new Cloud);
//                io::loadPCDFile(pcd_file_path, *cloud_temp);
//                vector<int> nan_index;
//                PointCloudHelper::removeNaNFromPointCloud(*cloud_temp, *cloud_temp, nan_index);
//                cloud_temp2->resize(cloud_temp->size());
//                for (int k = 0; k < cloud_temp->size(); k++) {
//                    cloud_temp2->points[k].x = cloud_temp->points[k].x;
//                    cloud_temp2->points[k].y = cloud_temp->points[k].y;
//                    cloud_temp2->points[k].z = cloud_temp->points[k].z;
//                    cloud_temp2->points[k].r = cloud_temp->points[k].r;
//                    cloud_temp2->points[k].g = cloud_temp->points[k].g;
//                    cloud_temp2->points[k].b = cloud_temp->points[k].b;
//                    cloud_temp2->points[k].a = cloud_temp->points[k].a;
//                }
//                if (stamp_rot_map.find(cur_stamp) == stamp_rot_map.end()) {
//                    cout << "frame stamp cannot find, ignore " << cur_stamp << endl;
//                    continue;
//                }
//                PointCloudHelper::remove_duplicate(cloud_temp2, 0.04);
//                PointCloudHelper::transformPointCloud(*cloud_temp2, *cloud_temp2, stamp_rot_map[cur_stamp]);
//
//                *frame_merged_cloud_vec[i] += *cloud_temp2;
//            }
//        }
//        //for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//        //    stringstream ss;
//        //    ss << i << ".pcd";
//        //    cout << "save file " << ss.str() << endl;
//        //    if (frame_merged_cloud_vec[i]->size() == 0) {
//        //        cout << "point cloud empty,skip" << endl;
//        //        continue;
//        //    }
//        //    io::savePCDFileBinary("D:\\"+ ss.str(), *frame_merged_cloud_vec[i]);
//        //}
//
//        int segment_count2 = 10;
//        for (int i = 0; i < frame_merged_cloud_vec.size() / segment_count2 + 1; i++) {
//            CloudPtr rpca_cloud(new Cloud);
//            int stamp_count = 0;
//            for (int j = i * segment_count2; j < i * segment_count2 + segment_count2; j++, stamp_count++) {
//                if (j >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                int before_length = rpca_cloud->size();
//                *rpca_cloud += *frame_merged_cloud_vec[j];
//                int after_length = rpca_cloud->size();
//                for (int index = before_length; index < after_length; index++) {
//                    rpca_cloud->points[index].stamp_id = stamp_count;
//                }
//            }
//            CloudPtr cloud_temp(new Cloud);
//            cout << "processing rpca...[" << i * segment_count2 << "," << i * segment_count2 + segment_count2 - 1 << "]";
//            tree_filter(rpca_cloud, cloud_temp);
//            //cloud_temp = rpca_cloud;
//            cout << "done." << endl;
//
//            vector<CloudPtr> cloud_temp_vec(segment_count2);
//            for (int j = 0; j < cloud_temp_vec.size(); j++) {
//                cloud_temp_vec[j] = boost::make_shared<Cloud>();
//            }
//            for (int j = 0; j < cloud_temp->size(); j++) {
//                int index = cloud_temp->points[j].stamp_id;
//                if (index >= 0 && index < segment_count2) {
//                    cloud_temp_vec[index]->points.push_back(cloud_temp->points[j]);
//                }
//            }
//            for (int j = 0; j < cloud_temp_vec.size(); j++) {
//                int index = i * segment_count2 + j;
//                if (index >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                *frame_merged_cloud_vec[index] = *cloud_temp_vec[j];
//            }
//            cloud_temp_vec.clear();
//        }
//#else
//#pragma omp parallel for
//        for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//            frame_merged_cloud_vec[i] = boost::make_shared<Cloud>();
//            stringstream ss;
//            ss << "Z:\\20160719\\frame_pca\\" << i << ".pcd";
//            cout << "load pcd " << ss.str() << endl;
//            io::loadPCDFile(ss.str(), *frame_merged_cloud_vec[i]);
//        }
//#endif
//
//        //for (int i = 0; i < frame_merged_cloud_vec.size(); i++) {
//        //    stringstream ss;
//        //    ss << i << ".pcd";
//        //    cout << "save file " << ss.str() << endl;
//        //    if (frame_merged_cloud_vec[i]->size() == 0) {
//        //        cout << "point cloud empty,skip" << endl;
//        //        continue;
//        //    }
//        //    io::savePCDFileBinary("D:\\"+ ss.str(), *frame_merged_cloud_vec[i]);
//        //}
//
//        int icp_cache_num = 10;
//        float icp_threshold = 0.13;
//        vector<Eigen::Matrix4d> rot_vec(frame_merged_cloud_vec.size());
//        for (int i = 0; i < rot_vec.size(); i++) {
//            rot_vec[i] = Eigen::Matrix4d::Zero();
//        }
//        for (int i = 0; i < frame_merged_cloud_vec.size() - icp_cache_num; i++) {
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                cout << seg_group[i][j];
//                if (j < seg_group[i].size() - 1) {
//                    cout << ",";
//                }
//            }
//            cout << endl;
//            CloudPtr cloud(new Cloud);
//            CloudPtr cloud_cache(new Cloud);
//            for (int cache_index = 0; cache_index < icp_cache_num; cache_index++) {
//                int cur_index = i + cache_index;
//                if (cur_index >= frame_merged_cloud_vec.size()) {
//                    break;
//                }
//                *cloud += *frame_merged_cloud_vec[cur_index];
//            }
//            PointCloudHelper::remove_duplicate(cloud, 0.04);
//            CloudItem min_xyz;
//            CloudItem max_xyz;
//            PointCloudHelper::getMinMax3D(*cloud, min_xyz, max_xyz);
//            float d_length = 30;
//            min_xyz.x -= d_length;
//            min_xyz.y -= d_length;
//            min_xyz.z -= d_length;
//            max_xyz.x += d_length;
//            max_xyz.y += d_length;
//            max_xyz.z += d_length;
//            CloudGrid::instance().get_cloud_with_pos(cloud_cache,
//                min_xyz, max_xyz);
//            Eigen::Matrix4d pair_transform = Eigen::Matrix4d::Identity();
//            cout << "before icp cloud1 " << cloud_cache->size() << " cloud2 " << cloud->size() << endl;
//
//            size_t cloud_point_threshold = 10000;
//            if (cloud_cache->size() < cloud_point_threshold || cloud->size() < cloud_point_threshold
//                || cloud->size() / (float)cloud_cache->size() >= 10) {
//                    cout << "reg: size unvalid identity stamp " << cloud->header.stamp << endl;
//                    //process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Identity(), cur_index, -2.0, icp_threshold);
//                    continue;
//            }
//
//            //float icp_dis = PointCloudHelper::get_rot_icp_filter(cloud_cache, cloud_tras, pair_transform, true, false);
//            float icp_dis = PointCloudHelper::get_rot_icp(cloud_cache, cloud, pair_transform, false, false);
//            cout << "reg: icp ret: " << icp_dis << endl;
//            cout << "reg: icp rot: " << endl << pair_transform << endl;
//            if (icp_dis < icp_threshold) {
//                rot_vec[i] = pair_transform;
//            }
//        }
//
//        bool is_failed = true;
//        Eigen::Matrix4d rot_first;
//        Eigen::Matrix4d rot_last;
//        int index_first = -1;
//        int index_last = -1;
//        for (int i = 0; i < rot_vec.size(); i++) {
//            if (rot_vec[i] != Eigen::Matrix4d::Zero()) {
//                is_failed = false;
//                rot_first = rot_vec[i];
//                index_first = i;
//                break;
//            }
//        }
//        if (is_failed) {
//            cout << "process failed" << endl;
//            return;
//        }
//
//        for (int i = rot_vec.size() - 1; i >= 0; i--) {
//            if (rot_vec[i] != Eigen::Matrix4d::Zero()) {
//                is_failed = false;
//                rot_last = rot_vec[i];
//                index_last = i;
//                break;
//            }
//        }
//
//        // 向前填充
//        if (index_first != 0) {
//            for (int i = 0; i < index_first; i++) {
//                rot_vec[i] = rot_first;
//            }
//        }
//        // 向后填充
//        for (int i = index_last + 1; i < rot_vec.size(); i++) {
//            rot_vec[i] = rot_last;
//        }
//        // 中间填充
//        for (int i = index_first; i < index_last; i++) {
//            if (rot_vec[i] == Eigen::Matrix4d::Zero()) {
//                rot_vec[i] = rot_vec[i - 1];
//            }
//        }
//
//        for (int i = 0; i < rot_vec.size(); i++) {
//            const Eigen::Matrix4d& rot = rot_vec[i];
//            for (int j = 0; j < seg_group[i].size(); j++) {
//                uint64_t cur_stamp = seg_group[i][j];
//                stamp_rot_map[cur_stamp] = rot * stamp_rot_map[cur_stamp];
//            }
//        }
    vector<CloudStampRot> rots;
    vector<CloudStampRot> rots1;
    string proj_str;
    string pose_file = "Z:\\chengdu\\193\\pose\\20160511T143650 - 副本.log";
    string pose_file1 = "Z:\\20160719\\pose_icp.log";
    FileHelper::load_file_rot_icp_hdmap(pose_file, rots, proj_str);
    FileHelper::load_file_rot_icp_hdmap(pose_file1, rots1, proj_str);
    double center_x = rots[0]._rot(0, 3);
    double center_y = rots[0]._rot(1, 3);
    double center_z = rots[0]._rot(2, 3);
    Cloud_FPtr track_cloud1(new Cloud_F);
    Cloud_FPtr track_cloud2(new Cloud_F);
    for (int i = 0; i < rots.size(); i++) {
        CloudItem_F p;
        p.x = rots[i]._rot(0, 3) - center_x;
        p.y = rots[i]._rot(1, 3) - center_y;
        p.z = rots[i]._rot(2, 3) - center_z;
        p.r = 255;
        p.g = 255;
        p.b = 255;
        track_cloud1->push_back(p);
    }
    for (int i = 0; i < rots.size(); i++) {
        CloudItem_F p;
        p.x = rots1[i]._rot(0, 3) - center_x;
        p.y = rots1[i]._rot(1, 3) - center_y;
        p.z = rots1[i]._rot(2, 3) - center_z;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        track_cloud2->push_back(p);
    }
    io::savePCDFileBinary("D:\\track1.pcd", *track_cloud1);
    io::savePCDFileBinary("D:\\track2.pcd", *track_cloud2);
}

int main_test(int argc, char** argv)
{
	PointCloudHelper::dpcd2fpcd(string(argv[1]));
    //test_point_cloud();

    //test_get_min_max();

    //test_transform();

    //test_remove_nan();

    //test_copy_point_cloud();

    //test_voxel_grid();

    //test_kd_tree();

    //test_double_io();

    //test_float_io();

    //test_icp();

    test();

    return 0;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END
