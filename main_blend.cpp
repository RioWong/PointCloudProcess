#include "stdafx.h"
#include <sstream>
#include <queue>
#include <iostream>
#include <fstream>
#include <map>
//#include <windows.h>
#include <time.h>
#include <sstream>

//#include "ficp_tools.h"
#include "cloud_grid.h"
#include "file_helper.h"
#include "string_helper.h"
#include "point_cloud_helper.h"
#include "process_status.h"
#include "main.h"
#include "params.h"

// 建立场景时自动进行回环检测做回环优化

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::cout;
using std::endl;
using std::list;
using std::string;
using std::map;
using std::stringstream;
using std::vector;
using std::ifstream;
using std::ofstream;

using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using boost::unordered_map;
//using pcl::PointXYZ;
//using pcl::PointCloud;
//using pcl::KdTreeFLANN;

const string CLOUD_NAME = "HDL";
const float RDUMPLICATE_VALUE = 0.2f;
const int processCacheMinNum = 5;
const float LOOPOUTSIDE = 60;
const float MAXTRACEDIS = 0.4f;
const float MAX_WIDTH_CORNER = 5;

ProcessStatus g_status;

void process_update_rot_icp(CloudStampRot& d_cur_cloud_stamp_rot, const Matrix4d& d_pair_transform, const int cur_index, const float icp_value, const float icp_threshold)
{
    if (icp_value >= 0 && icp_value < icp_threshold) {
        // 更新POS
        d_cur_cloud_stamp_rot._rot = d_pair_transform * g_status._global_transform;

        // 更新点云
        //pcl::transformPointCloud(*d_cur_cloud_stamp_rot._cloud_line, *d_cur_cloud_stamp_rot._cloud_line, d_pair_transform);

        PointCloudHelper::change_cloud_rgb(d_cur_cloud_stamp_rot._cloud_line, 0, 255, 0);
        // 加入到全局点云中
        //CloudGrid::instance().add_cloud_internal(d_cur_cloud_stamp_rot._cloud_line);

        // 加入到结果点云中
        //*g_status._cloud_result = *g_status._cloud_result + *d_cur_cloud_stamp_rot._cloud_line;
        //PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
        //PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    }
    // 写入POS文件
    FileHelper::write_file_rot_icp(cur_index, g_status._pcap_file,
                                   d_pair_transform * g_status._global_transform, true, d_cur_cloud_stamp_rot._stamp, icp_value);
}

void process_loop_icp(CloudPtr cloud_cache, CloudPtr cloud_tras, CloudPtr cloud_src, bool is_last_frame)
{
    //TODO global
    const float icp_threshold = 9.13;
    const uint64_t timestamp = cloud_tras->header.stamp;
    int cur_index = g_status.get_cloud_rot_index_with_stamp(timestamp);
    if (cur_index < 0) {
        cout << "reg: timestamp not found " << timestamp << endl;
        // ignore
        return;
    }

    CloudStampRot& cur_cloud_stamp_rot = g_status._clouds_stamp_rot_line[cur_index];
    cout << "loop start stamp: " << cloud_tras->header.stamp << endl;
    g_status._frame_count_process++;
    if (g_status._frame_count_process < g_status._num_start_frame_cache) {
        cout << "reg: frame ignore " << g_status._frame_count_process;
        process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Zero(), cur_index, -1.0, icp_threshold);
        return;
    }
    for (map<uint64_t, uint64_t>::iterator it = g_status._ignore_stamp.begin(); it != g_status._ignore_stamp.end(); ++it) {
        const uint64_t start_timestamp = it->first;
        const uint64_t end_timestamp = it->second;
        if (timestamp >= start_timestamp && timestamp <= end_timestamp) {
            cout << "reg: timestamp ignore" << endl;
            process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Identity(), cur_index, -2.0, icp_threshold);
            return;
        }
    }

    Matrix4d pair_transform = Matrix4d::Identity();
    cout << "before icp cloud1 " << cloud_cache->size() << " cloud2 " << cloud_tras->size() << endl;

    size_t cloud_point_threshold = 10000;
    if (cloud_cache->size() < cloud_point_threshold || cloud_tras->size() < cloud_point_threshold
        || cloud_tras->size() / (float)cloud_cache->size() >= 10) {
        cout << "reg: size unvalid identity stamp " << cloud_tras->header.stamp << endl;
        process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Identity(), cur_index, -2.0, icp_threshold);
        return;
    }

    //float icp_dis = PointCloudHelper::get_rot_icp_filter(cloud_cache, cloud_tras, pair_transform, true, false);
    float icp_dis = PointCloudHelper::get_rot_icp(cloud_cache, cloud_tras, pair_transform, false, false);
    cout << "reg: icp ret: " << icp_dis << endl;
    cout << "reg: icp rot: " << endl << pair_transform << endl;
    //io::savePCDFileBinary("D:\\base.pcd", *cloud_cache);
    //io::savePCDFileBinary("D:\\frame.pcd", *cloud_tras);
#if DEBUG_VIEW
    // 测试输出显示
    if (true) {
        CloudPtr temp1(new Cloud());
        CloudPtr temp2(new Cloud());
        *temp1 = *cloud_cache;
        pcl::transformPointCloud(*cloud_tras, *temp2, pair_transform);
        *temp2 += *cloud_cache;
        PclPairVisualizer::instance().update_cloud_left(temp1);
        PclPairVisualizer::instance().update_cloud_right(temp2);
        if (g_status._is_debug_mode) {
            PclPairVisualizer::instance().spin();
        } else {
            PclPairVisualizer::instance().spin_once();
        }
    }
#endif
    //TODO global
    g_status._global_transform = Matrix4d::Identity();
    process_update_rot_icp(cur_cloud_stamp_rot, pair_transform, cur_index, icp_dis, icp_threshold);
    return;

    if (icp_dis < 0) {
        cout << "reg: icp failed icp result " << icp_dis << " stamp " << cloud_tras->header.stamp << endl;
        cout << "icp failed, return" << endl;
        process_update_rot_icp(cur_cloud_stamp_rot, Matrix4d::Identity(), cur_index, -1.0, icp_threshold);
        return;
        //  } else {
        //// 对最终的全局变换矩阵进行判断
        //Matrix4f rot_temp = pair_transform * g_status._global_transform;
        //if (abs(rot_temp(2, 3)) >= 100.0) {
        //	cout<< "reg: global_transform delta_z >= 100 "<<rot_temp(2, 3)<<endl;
        //	cout<<"配准失败,直接返回"<<endl;
        //	process_update_rot_icp(cur_cloud_stamp_rot, Matrix4f::Identity(), cur_index, -1.0, icp_threshold);
        //	return;
        //}
    }

    Matrix4d d_pair_transform = pair_transform;
    process_update_rot_icp(cur_cloud_stamp_rot, d_pair_transform, cur_index, icp_dis, icp_threshold);
    // 更细全局变换矩阵
    if (icp_dis < icp_threshold && icp_dis >= 0) {
        g_status._global_transform = pair_transform * g_status._global_transform;
    }
}

void on_get_hdl_cloud_for_reg(CloudPtr& cloud_temp_src)
{
    //   CloudPtr cloud_tras=cloud;
    //CloudPtr cloud_temp_src=cloud;
    double d_yaw = 0.0;
    double d_pitch = 0.0;
    double d_roll = 0.0;
    g_status._frame_count_total++;
    cout << "frame count " << g_status._frame_count_total << " cloud size " << cloud_temp_src->size() << " stamp " << cloud_temp_src->header.stamp << endl;

    //if (cloud->size() <= 10)
    //{
    //	cout << ".................点云数量太少 " <<
    //		cloud->header.stamp << " " << cloud->size() << endl;
    //	fstream outfile;
    //	string filePath = g_status._pcap_file + "_mt_error.txt";
    //	outfile.open(filePath, ios::app);
    //	outfile << cloud->header.stamp << " " << cloud->size() << " 点云数量太少" << endl;
    //	outfile.close();
    //	return;
    //}
    //if (check_ignore_frame_valid(cloud->header.stamp / 1000, g_status._ignore_stamp) == false) {
    //	cout << "ignore frame ........." << cloud->header.stamp / 1000 << endl;
    //	return;
    //}
    //   

    ////恢复历史记录
    //if (g_status._start_frame > 0) {
    //	Matrix4f rot_old;
    //	CloudStampRot &stamp_rot=g_status.get_cloud_rot_with_stamp(cloud_temp_src->header.stamp);
    //	if (stamp_rot._is_valid) {
    //		rot_old=stamp_rot._rot;
    //			cout << "use old rot " << cloud_temp_src->header.stamp/* / 1000 */<< endl;
    //			g_status._global_transform = rot_old;
    //			if (PointCloudHelper::cloud_convert_with_rot(g_status._clouds_stamp_rot_line,
    //				cloud_temp_src->header.stamp/* / 1000*/, cloud_temp_src, cloud_tras,
    //				g_status._global_transform)) {
    //					//g_status._map_clouds_rot_stamps.erase(cloud->header.stamp / 1000);
    //			}
    //			CloudGrid::instance().add_cloud(cloud_tras);
    //			/*				CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
    //			g_status._global_transform)*/;
    //			//g_status._frame_count_process++;
    //			int num_per_show = 10;
    //			if (g_status._is_debug_mode)
    //			{
    //				num_per_show = 1;
    //			}else {
    //				if (g_status._is_batch_mode == true)
    //				{
    //					num_per_show = 50;
    //				}
    //			}
    //			if (g_status._frame_count_process % num_per_show == 0)
    //			{
    //				PclPairVisualizer::instance().update_cloud_right(cloud_tras);
    //			}
    //			process_end(cloud_temp_src, cloud_temp_src, cloud_tras, true);
    //			process_loop2(cloud_tras,cloud_tras,cloud_temp_src,false);
    //			return;
    //	}
    //	if (cloud_temp_src->header.stamp/* / 1000*/ < g_status._start_frame) {
    //           return;
    //       }
    //   }
    uint64_t cur_timestamp = cloud_temp_src->header.stamp;
    int last_index = g_status._clouds_stamp_rot_line.size() - 1;
    CloudPtr cloud_temp(new Cloud);
    *cloud_temp += *cloud_temp_src;
    for (int i = -g_status._num_start_frame_cache; i < 0; i++) {
        if (i + last_index < 0) {
            continue;
        }
        *cloud_temp += *g_status._clouds_stamp_rot_line[i + last_index]._cloud_line_src;
    }
    //TODO global
    //pcl::transformPointCloud(*cloud_temp, *cloud_temp, g_status._global_transform);
    cloud_temp->header.stamp = cur_timestamp;
    //io::savePCDFileBinary("D:\\1.pcd", *cloud_temp);
    //CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
    //	g_status._map_clouds_rot_stamps[cloud->header.stamp]);

    CloudItem min_xyz;
    CloudItem max_xyz;
    PointCloudHelper::getMinMax3D(*cloud_temp, min_xyz, max_xyz);
    float d_length = 30;
    min_xyz.x -= d_length;
    min_xyz.y -= d_length;
    min_xyz.z -= d_length;
    max_xyz.x += d_length;
    max_xyz.y += d_length;
    max_xyz.z += d_length;
    CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
                                             min_xyz, max_xyz);
    //CloudPtr cloud_temp2(new Cloud());
    //pcl::copyPointCloud(*cloud_temp, *cloud_temp2);
    //CloudGrid::instance().get_grid_cloud(cloud_temp, cloud_temp, g_status._cloud_cache);
    //PclPairVisualizer::instance().update_cloud_left(cloud_temp2);
    //PclPairVisualizer::instance().update_cloud_right(cloud_temp);
    //PclPairVisualizer::instance().spin();

    // TODO: 是否需要
    //PointCloudHelper::clear_trimesh_cache();

    //process_update_rot(trans_pair, trans_dst, cloud_temp_src, cloud_tras, cloud, d_yaw, 0);
    PointCloudHelper::remove_duplicate(cloud_temp, 0.04);
    //PointCloudHelper::remove_outlier(cloud_temp, 5000);
    //process_loop2(g_status._cloud_cache, cloud_temp,cloud_temp_src, false);
    process_loop_icp(g_status._cloud_cache, cloud_temp, cloud_temp_src, false);
}

class Rot
{
public:
    int ID;
    Eigen::Matrix4d _matrix;
    uint64_t TimeStamp;
    int _const100;
    double icperr;
    // 是否可靠
    bool is_valid;

    Rot(int id, const Eigen::Matrix4d mat, uint64_t timestamp, double icp_value)
        : ID(id), _matrix(mat), TimeStamp(timestamp), _const100(100), icperr(icp_value), is_valid(false)
    {
    }
};

std::string get_pcd_filepath_from_stamp(std::string dir_path, uint64_t timestamp)
{
    stringstream ss;
    ss << timestamp << ".pcd";
    return (boost::filesystem::path(dir_path) / ss.str()).string();
}

void FindReliable(vector<Rot>& Rots, std::string plypath, float icp_threshold) //找可靠点，若该相邻两矩阵的中心点乘相邻矩阵后距离不远，则判定可靠
{
#pragma omp parallel for
    for (int i = 1; i < Rots.size() - 1; i++) {
        //残差大于0小于0.13 
        if (Rots[i].icperr >= 0 && Rots[i].icperr <= icp_threshold) {
            //两边都是大于0的
            if (Rots[i - 1].icperr >= 0 && Rots[i + 1].icperr >= 0) {
                CloudItem leftpt, rightpt;
                std::string leftply_path, rightply_path, centraply_path; //相邻ply
                CloudItem minpt, maxpt;
                CloudPtr left_cloud(new Cloud);
                CloudPtr right_cloud(new Cloud);
                CloudPtr centra_cloud(new Cloud);
                leftply_path = get_pcd_filepath_from_stamp(plypath, Rots[i - 1].TimeStamp);
                //stringstream path;
                //int subply = Rots[i - 1].TimeStamp / 1000000;
                //int ply = Rots[i - 1].TimeStamp - subply * 1000000;
                //path << plypath << "\\" << subply << "_" << ply << ".ply";
                //leftply_path = path.str();
                io::loadPCDFile(leftply_path, *left_cloud); //加载三份ply
                //path.clear();
                //path.str("");
                //subply = Rots[i + 1].TimeStamp / 1000000;
                //ply = Rots[i + 1].TimeStamp - subply * 1000000;
                //path << plypath << "\\" << subply << "_" << ply << ".ply";
                //rightply_path = path.str();
                rightply_path = get_pcd_filepath_from_stamp(plypath, Rots[i + 1].TimeStamp);
                io::loadPCDFile(rightply_path, *right_cloud);
                //path.clear();
                //path.str("");
                //subply = Rots[i].TimeStamp / 1000000;
                //ply = Rots[i].TimeStamp - subply * 1000000;
                //path << plypath << "\\" << subply << "_" << ply << ".ply";
                //centraply_path = path.str();
                centraply_path = get_pcd_filepath_from_stamp(plypath, Rots[i].TimeStamp);
                cloud_blend_double::io::loadPCDFile(centraply_path, *centra_cloud);

                KdTreeFLANN<CloudItem> kdtree;
                //KdTree kdtree;
                kdtree.setInputCloud(centra_cloud);
                vector<int> pointIdxSearch;//存储邻域点的点号;
                vector<double> pointDistance;//存储邻域点到搜索点的距离;
                CloudItem searchPoint;//搜索点;
                double mix_dis = 9999;

                for (int j = 0; j < left_cloud->size(); j++) {
                    int N = kdtree.nearestKSearch(left_cloud->points[j], 1, pointIdxSearch, pointDistance);
                    if (pointDistance[0] < mix_dis) {
                        mix_dis = pointDistance[0];
                        searchPoint = left_cloud->points[j];
                    }
                }

                leftpt.x = searchPoint.x;
                leftpt.y = searchPoint.y;
                leftpt.z = searchPoint.z;
                CloudItem _leftpt, _rightpt;
                _leftpt.getVector4dMap() = Rots[i - 1]._matrix * leftpt.getVector4dMap();
                _rightpt.getVector4dMap() = Rots[i]._matrix * leftpt.getVector4dMap();
                double dis_left = PointCloudHelper::point_dis2(_leftpt, _rightpt);
                //double dis_left = Getdis(_leftpt, _rightpt);

                //算右边
                pointIdxSearch.clear();
                pointDistance.clear();
                mix_dis = 9999;
                for (int j = 0; j < right_cloud->size(); j++) {
                    int N = kdtree.nearestKSearch(right_cloud->points[j], 1, pointIdxSearch, pointDistance);
                    if (pointDistance[0] < mix_dis) {
                        mix_dis = pointDistance[0];
                        searchPoint = right_cloud->points[j];
                    }
                }

                rightpt.x = searchPoint.x;
                rightpt.y = searchPoint.y;
                rightpt.z = searchPoint.z;
                _leftpt.getVector4dMap() = Rots[i]._matrix * rightpt.getVector4dMap();
                _rightpt.getVector4dMap() = Rots[i + 1]._matrix * rightpt.getVector4dMap();
                double dis_right = PointCloudHelper::point_dis2(_leftpt, _rightpt);
#if DEBUG_OUTPUT
                cout << dis_left << " " << dis_right << " " << i << endl;
#endif
                /*   cout<<Rots[i].TimeStamp<<" "<<dis<<endl;*/
                float dis_threshold = 0.06;
                //两边连接处距离都小于0.06
                if (dis_left < dis_threshold && dis_right < dis_threshold) {
#if DEBUG_OUTPUT
                    cout << dis_left << " " << dis_right << " " << Rots[i].TimeStamp << endl;
#endif
                    //Rots[i].icperr=0.0;                       //icp err赋值为0，表示可靠
                    Rots[i].is_valid = true;
                }

            }
        }
    }
}

void generate_origin_poselog(const string& slice_stamp_file, const string& output_file, const vector<CloudStampRot>& rots, const string& hd_pose_path)
{
    ifstream ifile(slice_stamp_file.c_str());
    string d_vec_line;
    std::getline(ifile, d_vec_line);
    vector<string> d_vec_arr = StringHelper::split(d_vec_line, ",");
    for (int i = 0; i < 3; i++) {
        boost::trim(d_vec_arr[i]);
    }
    double x = boost::lexical_cast<double>(d_vec_arr[0]);
    double y = boost::lexical_cast<double>(d_vec_arr[1]);
    double z = boost::lexical_cast<double>(d_vec_arr[2]);
    Eigen::Vector3d d_vec(x, y, z);
    //cout.setf(ios::fixed);
    //cout << d_vec << endl;
    string line;
    vector<vector<string> > stamps_vec;
    while (!ifile.eof()) {
        std::getline(ifile, line);
        boost::trim(line);
        vector<string> vec_arr = StringHelper::split(line, " ");
        stamps_vec.push_back(vec_arr);
    }

    vector<CloudStampRot> hd_rots;
    string proj_str;
    FileHelper::load_file_rot_icp_hdmap(hd_pose_path, hd_rots, proj_str);
    unordered_map<string, Eigen::Matrix4d> stamp_d_rot;

    for (int i = 0; i < rots.size(); i++) {
        uint64_t stamp = rots[i]._stamp;
        const Eigen::Matrix4d& mat = rots[i]._rot;
        for (int j = 0; j < stamps_vec[stamp].size(); j++) {
            string stamp_d = stamps_vec[stamp][j];
            stamp_d_rot[stamp_d] = mat;
        }
    }

    ofstream ofile(output_file.c_str());
    ofile << proj_str << endl;
    ofile.close();
    unordered_map<string, Eigen::Matrix4d>::iterator end_it(stamp_d_rot.end());
    for (int i = 0; i < hd_rots.size(); i++) {
        string stamp_str = hd_rots[i]._time_stamp_str;
        unordered_map<string, Eigen::Matrix4d>::iterator it = stamp_d_rot.find(stamp_str);
        if (it == end_it) {
            continue;
        }
        const Eigen::Matrix4d& rot = it->second;
        Eigen::Matrix4d mat_d = hd_rots[i]._rot;
        //mat_d.block<3, 1>(0, 3) = mat_d.block<3, 1>(0, 3) - d_vec;
        Eigen::Matrix4d res_mat = Eigen::Matrix4d::Identity();
        res_mat.block<3, 3>(0, 0) = rot.block<3, 3>(0, 0) * mat_d.block<3, 3>(0, 0);
        res_mat.block<3, 1>(0, 3) = rot.block<3, 3>(0, 0) * mat_d.block<3, 1>(0, 3) - rot.block<3, 3>(0, 0) * d_vec + rot.block<3, 1>(0, 3) + d_vec;
        FileHelper::write_file_rot_hdmap(hd_rots[i]._stamp - 1, output_file, res_mat, true, stamp_str);
    }
}

int process_mode_pos(const vector<string>& files_pt_base, const vector<string>& files_pt_frame)
{
    if (files_pt_base.size() == 0) {
        cout << "error ............. base data empty" << endl;
        return -1;
    }
    g_status._global_transform = Matrix4d::Identity();
    //建立基准数据
    vector<CloudPtr> cloud_bases(files_pt_base.size());
    cout << "preparing data..." << endl;
#pragma omp parallel for
    for (int i = 0; i < files_pt_base.size(); ++i) {
#if DEBUG_OUTPUT
        cout << "start loading base point cloud data..." << files_pt_base[i] << endl;
#endif
        CloudPtr cloud_temp(new Cloud());
        io::loadPCDFile(files_pt_base[i], *cloud_temp);
        //pcl::io::loadPLYFile(files_pt_base[i], *cloud_temp);

        //PointCloudHelper::point_cloud_feature_filter(cloud_temp, cloud_temp);
        // 更改颜色
        PointCloudHelper::change_cloud_rgb(cloud_temp, 255, 255, 255);
        PointCloudHelper::remove_duplicate(cloud_temp, 0.04);
        cloud_bases[i] = cloud_temp;
    }
    CloudPtr cloud_base_temp(new Cloud);
    for (int i = 0; i < cloud_bases.size(); ++i) {
        *cloud_base_temp += *cloud_bases[i];
    }
    cloud_bases.clear();
    //PointCloudHelper::remove_duplicate(cloud_base_temp,0.04);
    CloudGrid::instance().add_cloud_internal(cloud_base_temp);
    cloud_base_temp->clear();
    //CloudPtr cloud_result(new Cloud);
    //CloudGrid::instance().get_grid_cloud(cloud_result);
    //io::savePCDFileBinary("D:\\1_jz.ply",*cloud_result);

#if DEBUG_VIEW
    PclPairVisualizer::instance()._pv->addCoordinateSystem(10, 0, 0, -5);
#endif
    //std::ofstream log((g_status._pcap_file + ".log").c_str());
    if (!g_status._is_batch_mode) {
        //std::streambuf * oldbuf = std::cout.rdbuf(log.rdbuf());
    }
    string hdl_calibration;
    //g_status._optimize_rot.clear();
    //加载历史rot
    boost::unordered_map<int, Eigen::Matrix4d> map_clouds_rot;
    boost::unordered_map<uint64_t, Eigen::Matrix4d> map_clouds_rot_stamps;
    FileHelper::load_file_rot(g_status._pcap_file, map_clouds_rot,
                              map_clouds_rot_stamps);
    unordered_map<uint64_t, Matrix4d>::iterator clouds_stamps_it(map_clouds_rot_stamps.begin());
    unordered_map<uint64_t, Matrix4d>::iterator clouds_stamps_end(map_clouds_rot_stamps.end());
    vector<uint64_t> map_clouds_stamps;
    for (; clouds_stamps_it != clouds_stamps_end; ++clouds_stamps_it) {
        map_clouds_stamps.push_back(clouds_stamps_it->first);
    }
    sort(map_clouds_stamps.begin(), map_clouds_stamps.end());

    if (boost::filesystem::exists(g_status._pcap_file) && boost::filesystem::is_regular_file(g_status._pcap_file)) {
    //if (map_clouds_rot_stamps.size() > 0) {
        namespace bf = boost::filesystem;
        bf::path path_rot(g_status._pcap_file);
        boost::filesystem::path pos_dir = path_rot.parent_path();
        boost::filesystem::path file_name = path_rot.filename().replace_extension();
        bf::path path_rot_old = pos_dir / (file_name.string() + "_" + StringHelper::get_time_string() + path_rot.extension().string());
        bf::copy_file(path_rot, path_rot_old);
        bf::remove(path_rot);
        for (unordered_map<uint64_t, Matrix4d>::iterator it =
             map_clouds_rot_stamps.begin();
             it != map_clouds_rot_stamps.end(); ++it) {
            if (it->second.block<3, 3>(0, 0) != Eigen::Matrix3d::Zero()) {
                if (g_status._start_frame < it->first) {
                    g_status._start_frame = it->first;
                }
            }
        }
    }
    //g_status._optimize_rot.clear();
    //加载忽略frame
    FileHelper::load_file_ignore_frame(g_status._pcap_file, g_status._ignore_stamp);

    for (int i = 0; i < files_pt_frame.size(); ++i) {
        // 原始点云
        CloudPtr cloud_src(new Cloud());

        uint64_t index, sub_index, current_stamp;
        //if (!FileHelper::get_index_stamp_from_filename(*it, index, sub_index, current_stamp)) {
        //    continue;
        //}
        if (!FileHelper::get_index_stamp_from_filename2(files_pt_frame[i], index, current_stamp)) {
            continue;
        }
        io::loadPCDFile(files_pt_frame[i], *cloud_src);
        //pcl::io::loadPLYFile(*it, *cloud_src);
        cloud_src->header.stamp = current_stamp;


        PointCloudHelper::change_cloud_rgb(cloud_src, 255, 0, 0);
        //pcl::transformPointCloud(*cloud_src, *cloud_trans, g_status._global_transform);
        //填充全局数据,以实际载入的数据为准,如果没有填入空白
        Matrix4d rot = map_clouds_rot_stamps.find(cloud_src->header.stamp) == map_clouds_rot_stamps.end() ? Eigen::Matrix4d::Zero() :
                           map_clouds_rot_stamps[cloud_src->header.stamp];
        CloudStampRot stamp_rot(cloud_src->header.stamp, rot, cloud_src, -3.0);
        g_status._clouds_stamp_rot_line.push_back(stamp_rot);

        cout << ".................................." << endl;
        cout << "load ply " << files_pt_frame[i] << endl;
#if DEBUG_VIEW
        if (g_status._frame_count_process==0) {
            PclPairVisualizer::instance()._pv->setCameraPose(cloud_src->points[0].x, cloud_src->points[0].y, cloud_src->points[0].z, 
                cloud_src->points[0].x, cloud_src->points[0].y, cloud_src->points[0].z - 500, 0, 0, 0);
        }
#endif

        // 加载历史记录
        if (false) {
            //if (rot != Matrix4f::Zero()) {
            cout << "load history " << cloud_src->header.stamp << endl;
            size_t cur_index = g_status._clouds_stamp_rot_line.size() - 1;
            CloudStampRot& d_cur_cloud_stamp_rot = g_status._clouds_stamp_rot_line[cur_index];
            // 更新POS
            d_cur_cloud_stamp_rot._rot = rot;

            // 更新点云
            PointCloudHelper::transformPointCloud(*cloud_src, *d_cur_cloud_stamp_rot._cloud_line, rot);

            PointCloudHelper::change_cloud_rgb(d_cur_cloud_stamp_rot._cloud_line, 0, 255, 0);
            // 加入到全局点云中
            //CloudGrid::instance().add_cloud_internal(d_cur_cloud_stamp_rot._cloud_line);

            // 加入到结果点云中
            //*g_status._cloud_result = *g_status._cloud_result + *d_cur_cloud_stamp_rot._cloud_line;
            //PointCloudHelper::remove_duplicate(g_status._cloud_result, 0.3);
            //PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);

            //CloudItem min_xyz;
            //CloudItem max_xyz;
            //pcl::getMinMax3D(*d_cur_cloud_stamp_rot._cloud_line, min_xyz, max_xyz);
            //float d_length =30;
            //min_xyz.x-=d_length;
            //min_xyz.y-=d_length;
            //min_xyz.z-=d_length;
            //max_xyz.x+=d_length;
            //max_xyz.y+=d_length;
            //max_xyz.z+=d_length;
            //CloudGrid::instance().get_cloud_with_pos(g_status._cloud_cache,
            //	min_xyz, max_xyz);
            //CloudPtr cloud_temp(new Cloud());
            //*cloud_temp += *d_cur_cloud_stamp_rot._cloud_line;
            //PointCloudHelper::change_cloud_rgb(cloud_temp, 255, 0, 0);
            //*cloud_temp += *g_status._cloud_cache;

            //PclPairVisualizer::instance().update_cloud_left(cloud_temp);
            //PclPairVisualizer::instance().spin();

            // 写入POS文件
            FileHelper::write_file_rot(cur_index, g_status._pcap_file,
                                       rot, true, d_cur_cloud_stamp_rot._stamp);
            g_status._global_transform = rot;
        } else {
            on_get_hdl_cloud_for_reg(cloud_src);
        }
        //CloudGrid::instance().add_cloud(cloud_temp);
        //清空不使用的点云
        g_status.clear_cloud_line_cache(30);
    }
    /*if (g_status._clouds_line.size() == 0) {
        cout << "pcap文件没有数据，疑似损坏" << endl;
        return 0;
    }*/
    // 对整个点云结束后还正在搜索的回环点进行处理
    //TODO global
    //process_last_icp();
    //process_loop2(g_status._cloud_cache, g_status._clouds_stamp_rot_line[g_status._clouds_stamp_rot_line.size() - 1]._cloud_line, true);
    //pcl::io::savePCDFile(g_status._pcap_file + "_loopsrc.pcd",
    //                     *g_status._cloud_result, false);
    return 0;
}

int process_mode_loop(
    const vector<string>& files_pt_base, 
    const vector<string>& files_pt_frame, 
    const string& dir_pt_frame, 
    const string& output_dir,
    const string& slice_stamp_file,
    const string& converted_pose_log,
    const string& output_pose_log)
{
    boost::filesystem::path output_path(output_dir);
    //boost::unordered_map< int, Eigen::Matrix4f> map_clouds_rot;
    //boost::unordered_map< uint64_t,Eigen::Matrix4f> map_clouds_rot_stamps;
    //加载历史rot
    //FileHelper::load_file_rot(g_status._pcap_file, map_clouds_rot,
    //                          map_clouds_rot_stamps);
    bool is_write_pose = false;
    if (g_status._clouds_stamp_rot_line.size() > 0) {
        is_write_pose = true;
    } else {
        CloudGrid::instance().clear();
        vector<CloudPtr> cloud_bases(files_pt_base.size());
        cout << "preparing data..." << endl;
#pragma omp parallel for
        for (int i = 0; i < files_pt_base.size(); ++i) {
#if DEBUG_OUTPUT
            cout << "start loading base point cloud data..." << files_pt_base[i] << endl;
#endif
            CloudPtr cloud_temp(new Cloud());
            io::loadPCDFile(files_pt_base[i], *cloud_temp);

            //PointCloudHelper::point_cloud_feature_filter(cloud_temp, cloud_temp);
            // 更改颜色
            PointCloudHelper::change_cloud_rgb(cloud_temp, 255, 255, 255);
            PointCloudHelper::remove_duplicate(cloud_temp, 0.04);
            cloud_bases[i] = cloud_temp;
            //#pragma omp critical
            //			{
            //				CloudGrid::instance().add_cloud_internal(cloud_temp);		
            //			}
        }
        CloudPtr cloud_base_temp(new Cloud);
        for (int i = 0; i < cloud_bases.size(); ++i) {
            *cloud_base_temp += *cloud_bases[i];
        }
        cloud_bases.clear();
        //PointCloudHelper::remove_duplicate(cloud_base_temp,0.04);
        CloudGrid::instance().add_cloud_internal(cloud_base_temp);
        cloud_base_temp->clear();
    }
    g_status._clouds_stamp_rot_line.clear();
    // TODO 如果是连续运行，则不清空rot_line直接使用，第一步的rot和icp_value要进行正确的赋值
    FileHelper::load_file_rot_icp(g_status._pcap_file, g_status._clouds_stamp_rot_line);
    //ProcessStatus &g = g_status;

    // 记录时间戳对应的文件名
    boost::unordered_map<uint64_t, string> stamp_filename_map;
    for (int i = 0; i < files_pt_frame.size(); i++) {
        uint64_t index = 0;
        uint64_t timestamp = 0;
        //if (!FileHelper::get_index_stamp_from_filename(files_pt_frame[i], index, sub_index, timestamp)) {
        if (!FileHelper::get_index_stamp_from_filename2(files_pt_frame[i], index, timestamp)) {
            continue;
        }
        stamp_filename_map[timestamp] = files_pt_frame[i];
    }

    if (true) {
        cout << "preparing data..." << endl;
        std::vector<Rot> rot_vec;
        for (int i = 0; i < g_status._clouds_stamp_rot_line.size(); i++) {
            uint64_t timestamp = g_status._clouds_stamp_rot_line[i]._stamp;
            Matrix4d mat = g_status._clouds_stamp_rot_line[i]._rot;
            double icp_value = g_status._clouds_stamp_rot_line[i]._value_icp;
            Rot rot(i, mat, timestamp, icp_value);
            rot_vec.push_back(rot);
        }
#if DEBUG_OUTPUT
        cout << "icp threshold " << g_status.min_icp_threshold << endl;
#endif
        FindReliable(rot_vec, dir_pt_frame, g_status.min_icp_threshold);

        // 将孤立的valid设置为invalid
        for (int i = 1; i < rot_vec.size() - 1; i++) {
            if (!rot_vec[i - 1].is_valid && !rot_vec[i + 1].is_valid && rot_vec[i].is_valid) {
                rot_vec[i].is_valid = false;
            }
        }
        for (int i = 0; i < rot_vec.size(); i++) {
            bool is_valid = rot_vec[i].is_valid;
            if (!is_valid) {
                g_status._clouds_stamp_rot_line[i]._value_icp = -1;
#if DEBUG_OUTPUT
                cout << "invalid " << g_status._clouds_stamp_rot_line[i]._stamp << endl;
#endif
            } else {
                g_status._clouds_stamp_rot_line[i]._value_icp = 0.001;
#if DEBUG_OUTPUT
                cout << "valid " << g_status._clouds_stamp_rot_line[i]._stamp << endl;
#endif
            }
        }
    }

    //检查是否有icp值
    bool is_contain_icp = false;
    if (g_status._clouds_stamp_rot_line.size() > 0) {
        // TODO: bug,第一帧ICP通常为-2
        is_contain_icp = (g_status._clouds_stamp_rot_line[0]._value_icp != -2);
    }
    // 
    if (is_contain_icp) {
        //根据阈值清理stamprot,将大于threshold的rot清零
        for (int i = 0; i < g_status._clouds_stamp_rot_line.size(); i++) {
            CloudStampRot& rot = g_status._clouds_stamp_rot_line[i];
            if (rot._value_icp > g_status.min_icp_threshold || rot._value_icp < 0) {
                rot._rot = Matrix4d::Zero();
            }
        }
        int first_none_zero_index = -1;
        // 查找第一个非零的index
        for (int j = 0; j + 1 < g_status._clouds_stamp_rot_line.size(); j++) {
            int cur_index = j;
            CloudStampRot& cur_stamp_rot = g_status._clouds_stamp_rot_line[cur_index];
            if (cur_stamp_rot._rot != Matrix4d::Zero()) {
                first_none_zero_index = cur_index;
                break;
            }
        }
        if (first_none_zero_index > 0) {
            // 将开始所有零矩阵都赋值为第一个非零矩阵的值
            for (int j = 0; j < first_none_zero_index; j++) {
                CloudStampRot& cur_stamp_rot = g_status._clouds_stamp_rot_line[j];
                cur_stamp_rot._rot = g_status._clouds_stamp_rot_line[first_none_zero_index]._rot;
            }
        }
        int last_none_zero_index = -1;
        for (int j = g_status._clouds_stamp_rot_line.size() - 1; j >= 0; j--) {
            int cur_index = j;
            CloudStampRot& cur_stamp_rot = g_status._clouds_stamp_rot_line[cur_index];
            if (cur_stamp_rot._rot != Matrix4d::Zero()) {
                last_none_zero_index = cur_index;
                break;
            }
        }
        if (last_none_zero_index > 0) {
            // 将末尾所有零矩阵都赋值为最后一个非零矩阵的值
            for (int j = last_none_zero_index + 1; j < g_status._clouds_stamp_rot_line.size(); j++) {
                CloudStampRot& cur_stamp_rot = g_status._clouds_stamp_rot_line[j];
                cur_stamp_rot._rot = g_status._clouds_stamp_rot_line[last_none_zero_index]._rot;
            }
        }
        for (int j = 0; j + 1 < g_status._clouds_stamp_rot_line.size();) {
            int cur_index = j;
            j++;
            CloudStampRot& cur_stamp_rot = g_status._clouds_stamp_rot_line[cur_index];
            if (cur_stamp_rot._rot == Matrix4d::Zero()) {
                continue;
            }
            int end_index = -1;
            for (int find_index = cur_index + 1; find_index < g_status._clouds_stamp_rot_line.size(); find_index++) {
                const Matrix4d& cur_pos = g_status._clouds_stamp_rot_line[find_index]._rot;

                // 若旋转矩阵为零矩阵
                if (cur_pos != Matrix4d::Zero()) {
                    end_index = find_index;
#if DEBUG_OUTPUT
                    cout << "loop: start_index found " << end_index << endl;
#endif
                    break;
                }
            }
            if (end_index < 0) {
                // 若前面的POS都是零矩阵
                for (size_t i = cur_index + 1; i < g_status._clouds_stamp_rot_line.size(); i++) {
                    CloudStampRot& d_cur_cloud_stamp_rot = g_status._clouds_stamp_rot_line[i];
                    // 赋值为当前的旋转矩阵
                    d_cur_cloud_stamp_rot._rot = cur_stamp_rot._rot;
                }
#if DEBUG_OUTPUT
                cout << "end_index not found." << endl;
#endif
                break;
            } else if (end_index - cur_index == 1) {
#if DEBUG_OUTPUT
                cout << "end_index - cur_index == 1" << endl;
#endif
                continue;
            }
            j = end_index;

            // 差分
            int length = end_index - cur_index + 1;
            const int max_invalid_count = 5;
            // 前后分别加载valid的帧数
            const int valid_count = 4;

            if (length >= max_invalid_count + 2) {
                // 如果连续5帧都是invalid，则进行大范围的ICP scale affine配准
#if DEBUG_OUTPUT
                cout << "start mul-frame ICP scale affine, length: " << length - 2 << endl;
                cout << "load mid" << endl;
#endif
                vector<uint64_t> update_rot_stamp;
                CloudPtr frame_cloud(new Cloud);
                vector<string> frame_files;
                for (int i = cur_index + 1; i < end_index; i++) {
                    const uint64_t timestamp = g_status._clouds_stamp_rot_line[i]._stamp;
                    unordered_map<uint64_t, string>::iterator find_it = stamp_filename_map.find(timestamp);
                    if (find_it == stamp_filename_map.end()) {
                        continue;
                    }
                    string file = find_it->second;
#if DEBUG_OUTPUT
                    cout << "find ply " << file << endl;
#endif
                    frame_files.push_back(file);
                    update_rot_stamp.push_back(timestamp);
                    //CloudPtr temp_cloud(new Cloud);
                    //pcl::io::loadPLYFile(file, *temp_cloud);
                    //*frame_cloud += *temp_cloud;
                }

#if DEBUG_OUTPUT
                cout << "load front" << endl;
#endif
                // 向前搜索
                int k = 0;
                int l = cur_index;
                int last_valid_index = -1;
                do {
                    if (l < 0) {
                        break;
                    }
                    const float icp_value = g_status._clouds_stamp_rot_line[l]._value_icp;
                    if (icp_value > 0 && icp_value < g_status.min_icp_threshold) {
                        k++;
                        if (last_valid_index < g_status._clouds_stamp_rot_line.size() && last_valid_index != l + 1) {
                            k = 0;
                        }
                        last_valid_index = l;
                    }
                    const uint64_t timestamp = g_status._clouds_stamp_rot_line[l]._stamp;
                    l--;
                    unordered_map<uint64_t, string>::iterator find_it = stamp_filename_map.find(timestamp);
                    if (find_it == stamp_filename_map.end()) {
                        continue;
                    }
                    string file = find_it->second;
#if DEBUG_OUTPUT
                    cout << "find ply " << file << endl;
#endif
                    frame_files.push_back(file);
                    update_rot_stamp.push_back(timestamp);
                    //CloudPtr temp_cloud(new Cloud);
                    //pcl::io::loadPLYFile(file, *temp_cloud);
                    //*frame_cloud += *temp_cloud;
                }
                while (k < valid_count);
#if DEBUG_OUTPUT
                cout << "load back" << endl << endl;
#endif
                // 向后搜索
                last_valid_index = -1;
                k = 0;
                l = end_index;
                do {
                    if (l >= g_status._clouds_stamp_rot_line.size()) {
                        break;
                    }
                    const float icp_value = g_status._clouds_stamp_rot_line[l]._value_icp;
                    if (icp_value > 0 && icp_value < g_status.min_icp_threshold) {
                        k++;
                        if (last_valid_index >= 0 && last_valid_index != l - 1) {
                            k = 0;
                        }
                        last_valid_index = l;
                    }
                    const uint64_t timestamp = g_status._clouds_stamp_rot_line[l]._stamp;
                    l++;
                    unordered_map<uint64_t, string>::iterator find_it = stamp_filename_map.find(timestamp);
                    if (find_it == stamp_filename_map.end()) {
                        continue;
                    }
                    string file = find_it->second;
#if DEBUG_OUTPUT
                    cout << "find ply " << file << endl;
#endif
                    frame_files.push_back(file);
                    update_rot_stamp.push_back(timestamp);
                    //CloudPtr temp_cloud(new Cloud);
                    //pcl::io::loadPLYFile(file, *temp_cloud);
                    //*frame_cloud += *temp_cloud;
                }
                while (k < valid_count);
#if DEBUG_OUTPUT
                cout << endl;
#endif

                vector<CloudPtr> frame_cloud_vec(frame_files.size());
#pragma omp parallel for
                for (int i = 0; i < frame_files.size(); i++) {
#if DEBUG_OUTPUT
                    cout << "load ply" << frame_files[i] << endl;
#endif
                    frame_cloud_vec[i] = boost::make_shared<Cloud>();
                    io::loadPCDFile(frame_files[i], *frame_cloud_vec[i]);
                    frame_cloud_vec[i]->header.stamp = update_rot_stamp[i];
                }

                for (int i = 0; i < frame_cloud_vec.size(); i++) {
                    *frame_cloud += *frame_cloud_vec[i];
                }
                //frame_cloud_vec.clear();

                PointCloudHelper::change_cloud_rgb(frame_cloud, 255, 0, 0);
                PointCloudHelper::remove_duplicate(frame_cloud, 0.04);
                //PointCloudHelper::remove_outlier(frame_cloud,10);

                cout << "frame_cloud->size() " << frame_cloud->size() << endl;

                CloudPtr cache_cloud(new Cloud);
                CloudItem min_xyz;
                CloudItem max_xyz;
                PointCloudHelper::getMinMax3D(*frame_cloud, min_xyz, max_xyz);
                float d_length = 30;
                min_xyz.x -= d_length;
                min_xyz.y -= d_length;
                min_xyz.z -= d_length;
                max_xyz.x += d_length;
                max_xyz.y += d_length;
                max_xyz.z += d_length;
                CloudGrid::instance().get_cloud_with_pos(cache_cloud,
                                                         min_xyz, max_xyz);
                cout << "cache_cloud->size() " << cache_cloud->size() << endl;
#if DEBUG_VIEW
                CloudPtr overall_cloud_ori(new Cloud);
                *overall_cloud_ori += *cache_cloud;
                *overall_cloud_ori += *frame_cloud;
                PclPairVisualizer::instance()._pv->setCameraPose(
                    frame_cloud->points[0].x, frame_cloud->points[0].y, frame_cloud->points[0].z + 20, 
                    frame_cloud->points[0].x, frame_cloud->points[0].y, frame_cloud->points[0].z, 
                    0, 0, 0);
                PclPairVisualizer::instance()._pv->registerKeyboardCallback(keyboard_callback);
                PclPairVisualizer::instance().update_cloud_left(overall_cloud_ori);
                if (g_status._is_debug_mode) {
                    PclPairVisualizer::instance().spin();
                } else {
                    PclPairVisualizer::instance().spin_once();
                }
#endif

                CloudPtr overall_cloud_out(new Cloud);
                Matrix4d rot = Matrix4d::Identity();
                float dis = PointCloudHelper::get_rot_icp(cache_cloud, frame_cloud, rot, true, false);
#if DEBUG_OUTPUT
                cout << dis << endl;
                cout << rot << endl;
                CloudPtr trans_cloud(new Cloud);
                CloudPtr trans_split_cloud(new Cloud);
                CloudPtr overall_cloud_final(new Cloud);
                PointCloudHelper::transformPointCloud(*frame_cloud, *trans_cloud, rot);
                *overall_cloud_final += *trans_cloud;
                *overall_cloud_final += *cache_cloud;

                // debug文件夹, 若不存在则建立
                boost::filesystem::path output_debug_path = output_path / "debug";
                if (!boost::filesystem::exists(output_debug_path) || !boost::filesystem::is_directory(output_debug_path)) {
                    boost::filesystem::create_directories(output_debug_path);
                }
                stringstream ss;
                ss << cur_stamp_rot._stamp;
                boost::filesystem::path prefix_path = output_debug_path / ss.str();
                cout << "saving debug file " << prefix_path.string() + "_0.pcd" << endl;
                cout << "cache_cloud->size() " << cache_cloud->size() << endl;
                io::savePCDFile(prefix_path.string() + "_0.pcd", *cache_cloud);

                cout << "saving debug file " << prefix_path.string() + "_1.pcd" << endl;
                cout << "frame_cloud->size() " << frame_cloud->size() << endl;
                io::savePCDFile(prefix_path.string() + "_1.pcd", *frame_cloud);

                cout << "saving debug file " << prefix_path.string() + "_2.pcd" << endl;
                cout << "trans_cloud->size() " << trans_cloud->size() << endl;
                io::savePCDFile(prefix_path.string() + "_2.pcd", *trans_cloud);
#endif
                for (int i = 0; i < update_rot_stamp.size(); i++) {
                    CloudStampRot& cloud_rot = g_status.get_cloud_rot_with_stamp(update_rot_stamp[i]);
                    CloudPtr cloud_temp(new Cloud);
                    CloudPtr cloud_src(new Cloud);
                    cloud_src->header.stamp = 0;
                    for (int k = 0; k < frame_files.size(); k++) {
                        if (frame_cloud_vec[k]->header.stamp == update_rot_stamp[i]) {
                            cloud_src = frame_cloud_vec[k];
                        }
                    }
                    if (cloud_src->header.stamp == 0) {
                        cout << "cannot find corres-stamp " << cloud_src->header.stamp << endl;
                        continue;
                    }
                    //pcl::transformPointCloud(*cloud_rot._cloud_line_src,*cloud_temp,rot);
                    Matrix4d rot_split;
                    PointCloudHelper::transformPointCloud(*cloud_src, *cloud_temp, rot);
                    CloudPtr cache_cloud_temp(new Cloud);
                    CloudGrid::instance().get_grid_cloud(cloud_temp, cloud_temp, cache_cloud_temp, 0.20);
                    dis = PointCloudHelper::get_rot_icp(cache_cloud_temp, cloud_temp, rot_split, false, false);
#if DEBUG_OUTPUT
                    io::savePCDFileBinary(prefix_path.string() + "_4.pcd", *cache_cloud_temp);
                    cout << "******split*****" << endl;
                    cout << dis << endl;
                    cout << rot_split << endl;
                    PointCloudHelper::transformPointCloud(*cloud_temp,*cloud_temp,rot_split);
                    *trans_split_cloud+=*cloud_temp;
#endif
                    rot_split = rot_split * rot;
                    cloud_rot._rot = rot_split;
                }
                frame_cloud_vec.clear();
#if DEBUG_OUTPUT
                io::savePCDFileBinary(prefix_path.string() + "_3.pcd", *trans_split_cloud);
#endif
#if DEBUG_VIEW
                PclPairVisualizer::instance().update_cloud_right(overall_cloud_final);
                if (g_status._is_debug_mode) {
                    PclPairVisualizer::instance().spin();
                } else {
                    PclPairVisualizer::instance().spin_once();
                }
#endif

                //for (int i = cur_index + 1; i < end_index; i++) {
                //    g_status._clouds_stamp_rot_line[i]._rot = rot;
                //}
            } else {
                if (true) {
#if DEBUG_OUTPUT
                    cout << "start lum-elch, length: " << length - 2 << endl;
#endif
                    // 如果连续1-4帧都是invalid，则进行误差分配
                    boost::shared_array<double> weights(new double[length]);
                    for (size_t i = 0; i < length; i++) {
                        weights[i] = double(i) / (length - 1);
                    }

                    Eigen::Matrix4d base_rot = cur_stamp_rot._rot;

                    Eigen::Matrix4d pair_transform = g_status._clouds_stamp_rot_line[end_index]._rot * base_rot.inverse();

                    // 误差分配(差分)
                    Eigen::Matrix4d loop_transform_double = pair_transform;
                    Eigen::Affine3d bl(loop_transform_double);
                    Eigen::Quaterniond q(bl.rotation());
                    Eigen::Matrix3d eps_mat
                        = loop_transform_double.block<3, 3>(0, 0) * bl.rotation().inverse() - Eigen::Matrix3d::Identity();
                    for (size_t i = cur_index; i <= end_index; i++) {
                        Eigen::Matrix3d scale_mat = Eigen::Matrix3d::Identity();
                        scale_mat += weights[i - cur_index] * eps_mat;
                        Eigen::Vector3d t2;
                        t2[0] = weights[i - cur_index] * loop_transform_double(0, 3);
                        t2[1] = weights[i - cur_index] * loop_transform_double(1, 3);
                        t2[2] = weights[i - cur_index] * loop_transform_double(2, 3);

                        Eigen::Quaterniond q2 = Eigen::Quaterniond::Identity().slerp(weights[i - cur_index], q);

                        Eigen::Translation3d t3(t2);
                        Eigen::Affine3d a(t3 * q2);
                        Eigen::Matrix4d each_trans_mat = Eigen::Matrix4d::Identity();
                        each_trans_mat = a * each_trans_mat;
                        each_trans_mat.block<3, 3>(0, 0) = scale_mat * each_trans_mat.block<3, 3>(0, 0);

                        // 差分矩阵
                        Eigen::Matrix4d d_pair_transform = each_trans_mat;
                        Eigen::Matrix4d res = d_pair_transform * base_rot;
                        //cout << d_pair_transform << endl;
                        //cout << res << endl;
                        //cout << endl;
                        g_status._clouds_stamp_rot_line[i]._rot = res;
                    } // 误差分配for
                } else {
                    // 直接使用前一帧的矩阵代替
                    for (size_t i = cur_index + 1; i < end_index; i++) {
                        g_status._clouds_stamp_rot_line[i]._rot = g_status._clouds_stamp_rot_line[cur_index]._rot;
                    }
                }
            } // if
        }
    }
    string pcap_file = g_status._pcap_file;
    boost::unordered_map<uint64_t, Eigen::Matrix4d> map_clouds_rot_stamps;
    for (size_t i = 0; i < g_status._clouds_stamp_rot_line.size(); i++) {
        const uint64_t timestamp = g_status._clouds_stamp_rot_line[i]._stamp;
        const Matrix4d& rot = g_status._clouds_stamp_rot_line[i]._rot;
        map_clouds_rot_stamps[timestamp] = rot;
    }

    // 读取frame点云
    // 使用前提 files_pt_frame是已经排序的列表
    int last_index = -1;
    uint64_t index = -1;
    CloudPtr cloud_output(new Cloud());
    if (!(boost::filesystem::exists(output_path) && boost::filesystem::is_directory(output_path))) {
        boost::filesystem::create_directories(output_path);
    }
    for (size_t i = 0; i < files_pt_frame.size(); ++i) {

        boost::filesystem3::path temp_path(files_pt_frame[i]);
        std::string filename = temp_path.filename().string();
        uint64_t sub_index = 0;
        uint64_t timestamp;
        //if (!FileHelper::get_index_stamp_from_filename(filename, index, sub_index, timestamp)) {
        if (!FileHelper::get_index_stamp_from_filename2(filename, index, timestamp)) {
            continue;
        }

        // 目的生成文件如果存在则跳过
        std::string save_file_temp = (output_path / (boost::format("0_%1%.pcd") % index).str()).string();
        if (boost::filesystem::exists(save_file_temp)) {
            continue;
        }
        if (last_index == -1) {
            last_index = index;
        }
        if (index != last_index) {
            if (cloud_output->size() != 0) {
                std::string save_file = (output_path / (boost::format("0_%1%.pcd") % last_index).str()).string();
                cout << "saving pcd file " << save_file << endl;
                io::savePCDFileBinary(save_file, *cloud_output);
                cloud_output->clear();
            }
            last_index = index;
        }


        if (map_clouds_rot_stamps.find(timestamp) == map_clouds_rot_stamps.end()) {
            //				cout<<"无法找到对应的时间戳,场景结束 stamp"<<timestamp<<endl;
            //				return 1;
            cout << "cannot find coress-stamp, skip stamp" << timestamp << endl;
            continue;
        }
        CloudPtr cloud_temp(new Cloud());
        io::loadPCDFile(files_pt_frame[i], *cloud_temp);
        cloud_temp->header.stamp = timestamp;

        const Matrix4d& rot = map_clouds_rot_stamps[timestamp];
#if DEBUG_OUTPUT
        cout << files_pt_frame[i] << endl;
        cout << rot << endl;
        boost::filesystem::path pos_file(g_status._pcap_file);
        boost::filesystem::path pos_dir = pos_file.parent_path();
        boost::filesystem::path file_name = pos_file.filename().replace_extension();
        string final_pos_file = (pos_dir / (file_name.string() + "_final" + pos_file.extension().string())).string();
        FileHelper::write_file_rot(i, final_pos_file, rot, i != 0, timestamp);
#else
        FileHelper::write_file_rot(i, g_status._pcap_file, rot, i != 0, timestamp);
#endif

        CloudPtr cloud_res(new Cloud());
        PointCloudHelper::transformPointCloud(*cloud_temp, *cloud_res, rot);
        *cloud_output += *cloud_res;
    }
    if (cloud_output->size() > 0 && index != -1) {
        std::string save_file = (output_path / (boost::format("0_%1%.pcd") % index).str()).string();
        cout << "saving pcd file " << save_file << endl;
        io::savePCDFileBinary(save_file, *cloud_output);
        cloud_output->clear();
    }
    vector<CloudStampRot> rots;
#if !DEBUG_OUTPUT
    FileHelper::load_file_rot_icp(g_status._pcap_file, rots);
    generate_origin_poselog(slice_stamp_file, output_pose_log, rots, converted_pose_log);
#else
    boost::filesystem::path pos_file(output_pose_log);
    boost::filesystem::path pos_dir = pos_file.parent_path();
    boost::filesystem::path file_name = pos_file.filename().replace_extension();
    string new_pose_file = (pos_dir / (file_name.string() + "_pose" + pos_file.extension().string())).string();
    string final_pos_file = (pos_dir / (file_name.string() + "_final" + pos_file.extension().string())).string();
    FileHelper::load_file_rot_icp(final_pos_file, rots);
    generate_origin_poselog(slice_stamp_file, new_pose_file, rots, converted_pose_log);
#endif
    return 0;
}

int main_blend(const enumModeProcess current_mode, const blend_params& params)
{
    //string dir_pt_base = "E:\\zhangyuzhi\\gaojin\\119\\pointcloud_ply_out_temp";
    //string dir_pt_frame = "\\\\172.18.2.219\\d\\新建文件夹\\OutSidePointCloud\\1201";

    string dir_pt_base = "D:\\WorkNew\\pointcloudblend\\testdata\\119\\pointcloud_ply_out";
    string dir_pt_frame = "\\\\172.18.2.219\\d\\gaojingmap\\OutSidePointCloud\\1202";
    g_status._pcap_file = "D:\\WorkNew\\pointcloudblend\\testdata\\119_120.pcap";
    g_status.min_icp_threshold = 0;

    // 第二步输出文件夹
    string output_path("E:\\zhangyuzhi\\gaojin\\out");
    string slice_stamp_file;
    string converted_pose_log;
    string output_pose_log;
    g_status._current_mode = ENUM_MODE_PROCESS_POS;
    if (current_mode == ENUM_MODE_PROCESS_POS) {
        g_status._current_mode = current_mode;
        dir_pt_base = params.base_dir;
        dir_pt_frame = params.frame_dir;
        g_status._pcap_file = params.out_pose_file;
        g_status._num_start_frame_cache = params.cache_frame_num;
    } else if (current_mode == ENUM_MODE_PROCESS_LOOP) {
        g_status._current_mode = current_mode;
        dir_pt_base = params.base_dir;
        dir_pt_frame = params.frame_dir;
        g_status._pcap_file = params.out_pose_file;
        output_path = params.cloud_dir;
        g_status.min_icp_threshold = params.min_icp_threshold;
        slice_stamp_file = params.stamp_file;
        converted_pose_log = params.in_pose_file;
        output_pose_log = params.out_pose_file;
        if (!(boost::filesystem::exists(output_path) && boost::filesystem::is_directory(output_path))){
            boost::filesystem::create_directories(output_path);
        }
    }


    vector<string> files_pt_base;
    vector<string> files_pt_frame;
    files_pt_base = FileHelper::get_all_ply_files2(dir_pt_base);
    files_pt_frame = FileHelper::get_all_ply_files2(dir_pt_frame);

    // PCD切分目录下的slice_stamp，用于还原原始的POSE文件

#if DEBUG_VIEW
    PclPairVisualizer::instance().update_cloud_left(g_status._cloud_result);
    PclPairVisualizer::instance().update_cloud_right(g_status._cloud_result);
    PclPairVisualizer::instance()._pv->registerKeyboardCallback(keyboard_callback);
#endif
    if (g_status._current_mode == ENUM_MODE_PROCESS_POS) {
        process_mode_pos(files_pt_base, files_pt_frame);
    } else if (g_status._current_mode == ENUM_MODE_PROCESS_LOOP) {
        process_mode_loop(files_pt_base, files_pt_frame, dir_pt_frame, output_path, slice_stamp_file, converted_pose_log, output_pose_log);
    }

    return 0;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END
