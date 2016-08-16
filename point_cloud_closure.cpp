#include "stdafx.h"
#include "point_cloud_closure.h"
#include "point_cloud_spliter.h"
#include "main.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

using std::cout;
using std::endl;
using std::ofstream;
using std::stringstream;
using std::vector;
using std::string;
using std::pair;
using boost::unordered_map;

PointCloudClosure::PointCloudClosure(void)
{
}


PointCloudClosure::~PointCloudClosure(void)
{
}

double PointCloudClosure::distance_sqr_rot(const Eigen::Matrix4d &rot1, const Eigen::Matrix4d &rot2)
{
    double x1 = rot1(0, 3);
    double y1 = rot1(1, 3);
    double z1 = rot1(2, 3);

    double x2 = rot2(0, 3);
    double y2 = rot2(1, 3);
    double z2 = rot2(2, 3);

    return std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2);
}

uint64_t PointCloudClosure::minus_abs(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

void PointCloudClosure::get_overlap_stamp(const vector<CloudStampRot> &rots, vector<same_segment> &overlap_segs)
{
    vector<PointC> d_rots;
    PointCloud<PointC>::Ptr cloud_d_rots(new PointCloud<PointC>);

    for (int i = 0; i < rots.size() - 1; i++) {
        double x = rots[i]._rot(0, 3);
        double y = rots[i]._rot(1, 3);
        double z = rots[i]._rot(2, 3);
        double d_x = rots[i + 1]._rot(0, 3) - x;
        double d_y = rots[i + 1]._rot(1, 3) - y;
        double d_z = rots[i + 1]._rot(2, 3) - z;
        PointC d_pos;
        d_pos.x = x;
        d_pos.y = y;
        d_pos.z = z;
        d_pos.stamp = rots[i]._stamp;
        d_pos.angle_z = RAD2DEG(atan2(d_y, d_x));
        d_pos.distance_sqr = std::pow(d_x, 2) + std::pow(d_y, 2) + std::pow(d_z, 2);

        if (d_pos.angle_z < 0) {
            d_pos.angle_z += 360.0f;
        }

        if (d_pos.distance_sqr <= 0.1) {
            continue;
        }

        //cout << d_pos.angle_z << endl;
        //cout << d_pos.angle_z << "," <<
        //    RAD2DEG(atan2(-rots[i]._rot(2, 0), rots[i]._rot(0, 0)))
        //    << endl;
        d_rots.push_back(d_pos);
        cloud_d_rots->push_back(d_pos);
    }

    Cloud_FPtr track_cloud1(new Cloud_F);
    //Cloud_FPtr track_cloud2(new Cloud_F);
    KdTreeFLANN<PointC> kd_tree;
    kd_tree.setInputCloud(cloud_d_rots);
    vector<pair<uint64_t, uint64_t> > pair_vec;

    for (int i = 0; i < cloud_d_rots->size(); i++) {
        vector<int> index;
        vector<double> distance_sqr;
        const PointC& cur_p = cloud_d_rots->points[i];
        const uint64_t cur_stamp = cur_p.stamp;
        int length = kd_tree.radiusSearch(cur_p, 10, index, distance_sqr);        bool is_find = false;
        int find_index = -1;

        for (int j = 0; j < length; j++) {
            const uint64_t stamp = cloud_d_rots->points[index[j]].stamp;

            if (stamp <= cur_stamp) {
                continue;
            }

            const uint64_t abs_stamp = minus_abs(cur_stamp, stamp);

            if (abs_stamp < 6000) {
                continue;
            }

            double a1 = cur_p.angle_z;
            double a2 = cloud_d_rots->points[index[j]].angle_z;
            double d_angle = std::abs(a1 - a2);
            //double d_angle = std::min(std::abs(a1 - a2), std::abs(a1 + a2 - 360));

            if (d_angle >= 20.0f) {
                continue;
            }

			//求沿行驶方向上的水平距离不能超过1.5
			double avg_angle = (a1 + a2) / 2.0;
			avg_angle -= 90;
			avg_angle = avg_angle/180.0*M_PI;
			double k = tan(avg_angle);
			//直线ax+by+c=0
			double b= cur_p.y-k*cur_p.x;
			double dis=std::abs(0-k*cloud_d_rots->points[index[j]].x + cloud_d_rots->points[index[j]].y - b)/std::sqrt(k*k+1);
			if (dis>1.5)
			{
                cout<<cur_stamp  << "<--->" << cloud_d_rots->points[index[j]].stamp << " dis=" << dis << " > 1.5"<<endl;
				continue;
			}

            is_find = true;
            find_index = index[j];
            break;
        }

        if (is_find) {
            cout << cur_stamp << "<--->" << cloud_d_rots->points[find_index].stamp << endl;
            pair_vec.push_back(std::make_pair<uint64_t, uint64_t>(cur_stamp,
                               cloud_d_rots->points[find_index].stamp));
        }
    }

    vector<same_segment> &same_segment_vec = overlap_segs;
    same_segment_vec.clear();
    if (pair_vec.size() > 0) {
        uint64_t base_start = pair_vec[0].first;
        uint64_t frame_start = pair_vec[0].second;

        for (int i = 1; i < pair_vec.size(); i++) {
            uint64_t base_gap = minus_abs(pair_vec[i].first, pair_vec[i - 1].first);
            uint64_t frame_gap = minus_abs(pair_vec[i].second, pair_vec[i - 1].second);

            if (base_gap > 10 || frame_gap > 10) {
                same_segment seg;
                seg.base_start_stamp = base_start;
                seg.base_end_stamp = pair_vec[i - 1].first;
                seg.frame_start_stamp = frame_start;
                seg.frame_end_stamp = pair_vec[i - 1].second;
                same_segment_vec.push_back(seg);
                //cout << "base_start " << base_start << " " << "frame_start " << frame_start << "base_end " <<
                //     pair_vec[i - 1].first << " " << "frame_end " << pair_vec[i - 1].second << endl;
                base_start = pair_vec[i].first;
                frame_start = pair_vec[i].second;
            }
        }

        same_segment seg;
        seg.base_start_stamp = base_start;
        seg.base_end_stamp = pair_vec[pair_vec.size() - 1].first;
        seg.frame_start_stamp = frame_start;
        seg.frame_end_stamp = pair_vec[pair_vec.size() - 1].second;
        same_segment_vec.push_back(seg);
        for (int i = 0; i < same_segment_vec.size(); i++) {
            uint64_t &frame_start_stamp = same_segment_vec[i].frame_start_stamp;
            uint64_t &frame_end_stamp = same_segment_vec[i].frame_end_stamp;
            if (frame_start_stamp > frame_end_stamp) {
                std::swap(frame_start_stamp, frame_end_stamp);
            }
        }
    }
}

int PointCloudClosure::get_index_from_rots(const vector<CloudStampRot> &rots, uint64_t stamp) 
{
    int index = -1;
    for (int i = 0; i < rots.size(); i++) {
        if (stamp == rots[i]._stamp) {
            index = i;
            break;
        }
    }
    return index;
}

bool PointCloudClosure::do_lum_elch(vector<CloudStampRot> &rots, int start_index, int end_index, const Eigen::Matrix4d &loop_transform)
{
    int length = end_index - start_index + 1;
    // 如果连续1-4帧都是invalid，则进行误差分配
    boost::shared_array<double> weights(new double[length]);
    for (size_t i = 0; i < length; i++) {
        weights[i] = double(i) / (length - 1);
    }

    // 误差分配(差分)
    Eigen::Affine3d bl(loop_transform);
    Eigen::Quaterniond q(bl.rotation());
    Eigen::Matrix3d eps_mat
        = loop_transform.block<3, 3>(0, 0) * bl.rotation().inverse() - Eigen::Matrix3d::Identity();
    for (size_t i = start_index; i <= end_index; i++) {
        Eigen::Matrix3d scale_mat = Eigen::Matrix3d::Identity();
        scale_mat += weights[i - start_index] * eps_mat;
        Eigen::Vector3d t2;
        t2[0] = weights[i - start_index] * loop_transform(0, 3);
        t2[1] = weights[i - start_index] * loop_transform(1, 3);
        t2[2] = weights[i - start_index] * loop_transform(2, 3);

        Eigen::Quaterniond q2 = Eigen::Quaterniond::Identity().slerp(weights[i - start_index], q);

        Eigen::Translation3d t3(t2);
        Eigen::Affine3d a(t3 * q2);
        Eigen::Matrix4d each_trans_mat = Eigen::Matrix4d::Identity();
        each_trans_mat = a * each_trans_mat;
        each_trans_mat.block<3, 3>(0, 0) = scale_mat * each_trans_mat.block<3, 3>(0, 0);

        // 差分矩阵
        Eigen::Matrix4d d_pair_transform = each_trans_mat;
        //cout << d_pair_transform << endl;
        //cout << d_pair_transform << endl;
        //cout << res << endl;
        //cout << endl;
        rots[i]._rot = d_pair_transform * rots[i]._rot;
    } // 误差分配for
    return true;
}

bool PointCloudClosure::do_loop_closure(vector<CloudStampRot> &ori_rots, const vector<CloudStampRot> &opt_rots)
{
    uint64_t start_stamp = opt_rots[0]._stamp;
    uint64_t end_stamp = opt_rots[opt_rots.size() - 1]._stamp;
    int opt_index_start = get_index_from_rots(ori_rots, start_stamp);
    int opt_index_end = get_index_from_rots(ori_rots, end_stamp);
    if (opt_index_start < 0) {
        cout << "cannot find stamp in rots " << start_stamp << endl;
        return false;
    }
    if (opt_index_end < 0) {
        cout << "cannot find stamp in rots " << opt_index_end << endl;
        return false;
    }

    if (opt_index_end - opt_index_start + 1 != opt_rots.size()) {
        cout << "rots count not equal " << opt_index_end - opt_index_start + 1 << "," << opt_rots.size() << endl;
        return false;
    }

    Eigen::Matrix4d pair_transform = opt_rots[0]._rot * ori_rots[opt_index_start]._rot.inverse();
    Eigen::Matrix4d end_loop_transform = opt_rots[opt_rots.size() - 1]._rot * ori_rots[opt_index_end]._rot.inverse();
    cout << "start loop transform" << endl << pair_transform << endl;
    cout << "end loop transform" << endl << end_loop_transform << endl;
    const int length = 400;
    int start_index = opt_index_start - length + 1;
    if (start_index < 0) {
        start_index = 0;
    }
    int end_index = opt_index_start;
    cout << "processing loop closure" << endl;
    do_lum_elch(ori_rots, start_index, end_index, pair_transform);

    for (int i = opt_index_start; i <= opt_index_end; i++) {
        ori_rots[i]._rot = opt_rots[i - opt_index_start]._rot;
    }

    for (int i = opt_index_end + 1; i < ori_rots.size(); i++) {
        ori_rots[i]._rot = end_loop_transform * ori_rots[i]._rot;
    }
    return true;
}

bool PointCloudClosure::cleanup_pcd_in_dir(boost::filesystem::path dir)
{
    namespace fs = boost::filesystem;
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        return false;
    }
    fs::directory_iterator it(dir);
    fs::directory_iterator it_end;
    int count = 0;
    for (; it != it_end; ++it) {
        const fs::path &cur_path = it->path();
        if (!fs::is_regular_file(cur_path)) {
            continue;
        }
        if (boost::to_lower_copy(cur_path.extension().string()) == string(".pcd")) {
            fs::remove(cur_path);
            count++;
        }
    }
    if (count == 0) {
        return false;
    } else {
        return true;
    }
}

int PointCloudClosure::main(const closure_params &params)
{
    vector<CloudStampRot> rots;
    //string pose_file = "Z:\\107\\107.log";
    //string pose_file = "Z:\\106\\106.log";
    //string pose_file = "Z:\\120\\pose_log\\120_.log";
    //string pose_file = "Z:\\119\\pose_log\\119_.log";
    string pose_file = "Z:\\chengdu\\193\\pose\\20160511T143650 - 副本.log";
    // 需要根据pcd中的文件对rots中的stamp进行清理
    string pcd_dir = "Z:\\chengdu\\193\\pcd";
    string base_split_dir = "Z:\\20160719\\base_split";
    string frame_split_dir = "Z:\\20160719\\frame_split";
    string out_cloud_dir = "Z:\\20160719\\frame_split_out";
    string new_pose_file = "Z:\\20160719\\frame_split_out\\_pose.log";

    pose_file = params.in_origin_pose_file;
    pcd_dir = params.in_pcd_dir;
    base_split_dir = params.out_base_split_dir;
    frame_split_dir = params.out_frame_split_dir;
    out_cloud_dir = params.out_cloud_dir;
    new_pose_file = params.out_new_pose_file;

    string proj_str;
    FileHelper::load_file_rot_icp_hdmap(pose_file, rots, proj_str);

    vector<same_segment> same_segment_vec;
    get_overlap_stamp(rots, same_segment_vec);

    namespace fs = boost::filesystem;
    if (same_segment_vec.size() == 0) {
        if (fs::exists(pose_file) && fs::is_regular_file(pose_file)) {
            fs::copy_file(pose_file, new_pose_file, fs::copy_option::overwrite_if_exists);
            cout << "file copied(overwrite) from " << pose_file << " to " << new_pose_file << endl;
        }
        cout << "no overlap, exit" << endl;
        return 1;
    }

    fs::path pcd_dir_path(pcd_dir);
    fs::path base_split_path(base_split_dir);
    fs::path frame_split_path(frame_split_dir);
    for (int seg_index = 0; seg_index < same_segment_vec.size(); seg_index++) {
        uint64_t base_start_stamp = same_segment_vec[seg_index].base_start_stamp;
        uint64_t base_end_stamp = same_segment_vec[seg_index].base_end_stamp;
        uint64_t frame_start_stamp = same_segment_vec[seg_index].frame_start_stamp;
        uint64_t frame_end_stamp = same_segment_vec[seg_index].frame_end_stamp;

        cout << "base start " << base_start_stamp << " base end " << base_end_stamp << endl;
        cout << "frame start " << frame_start_stamp << " frame end " << frame_end_stamp << endl;

        if (minus_abs(frame_end_stamp, frame_start_stamp) < 10) {
            continue;
        }

        if (cleanup_pcd_in_dir(frame_split_path)) {
            cout << "pcd files in dir removed " << frame_split_path << endl;
        }
        if (cleanup_pcd_in_dir(base_split_path)) {
            cout << "pcd files in dir removed " << base_split_path << endl;
        }

        string base_stamp_file = (base_split_path / "stamp.log").string();
        string base_pose_file = (base_split_path / "pose.log").string();
        split_params base_params(pcd_dir, pose_file, pose_file, base_split_dir, base_pose_file, base_stamp_file);
        base_params.is_set_stamp = true;
        base_params.start_stamp = base_start_stamp;
        base_params.end_stamp = base_end_stamp;

        string frame_stamp_file = (frame_split_path / "stamp.log").string();
        string frame_pose_file = (frame_split_path / "pose.log").string();
        split_params frame_params(pcd_dir, pose_file, pose_file, frame_split_dir, frame_pose_file, frame_stamp_file);
        frame_params.is_set_stamp = true;
        frame_params.start_stamp = frame_start_stamp;
        frame_params.end_stamp = frame_end_stamp;

        int ret = 0;
        ret = PointCloudSpliter::main(base_params);
        if (ret == 0) {
            cout << "split error" << endl;
            return -1;
        }
        cout << "split base done." << endl;
        ret = PointCloudSpliter::main(frame_params);
        if (ret == 0) {
            cout << "split error" << endl;
            return -1;
        }
        cout << "split frame done." << endl;

        blend_params blend_parameter(base_split_dir, frame_split_dir, frame_stamp_file, frame_pose_file, out_cloud_dir, new_pose_file, true, 0.13, 10);
        ret = main_blend(ENUM_MODE_PROCESS_POS, blend_parameter);
        if (ret != 0) {
            return -1;
        }
        ret = main_blend(ENUM_MODE_PROCESS_LOOP, blend_parameter);
        if (ret != 0) {
            return -1;
        }

        vector<CloudStampRot> opt_rots;
        string opt_proj;
        FileHelper::load_file_rot_icp_hdmap(new_pose_file, opt_rots, opt_proj);
        if (opt_rots.size() == 0) {
            cout << "load pose file error " << new_pose_file << endl;
            return -1;
        }

        if (!do_loop_closure(rots, opt_rots)) {
            cout << "error, exit" << endl;
            return -1;
        }

//        if (true) {
//            if (boost::filesystem::exists(new_pose_file)) {
//                boost::filesystem::path src_file(new_pose_file);
//                boost::filesystem::path dst_file = src_file.parent_path() / (src_file.filename().replace_extension().string() + "_icp" + src_file.extension().string());
//                boost::filesystem::copy_file(src_file, dst_file);
//            }
//        }
#if DEBUG_OUTPUT
        FileHelper::copy_to_file_subfix(new_pose_file, "closure");
#endif

        std::ofstream ofile;
        ofile.open(new_pose_file.c_str(), std::ios::out | std::ios::trunc);
        ofile << proj_str << endl;
        ofile.close();
        cout << "writing file " << new_pose_file << endl;
        FileHelper::write_file_rot_hdmap(new_pose_file, rots, true);
        cout << "loop closure done" << endl;

    }
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END