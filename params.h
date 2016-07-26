#pragma once
#include <string>
#include "macros.h"
#include "process_status.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
struct split_params 
{
    // 待切割的PCD点云文件夹
    const std::string pcd_dir;

    // 基准的POSE文件
    const std::string base_rot_file;

    // 待配准的POSE文件
    const std::string frame_rot_file;

    // 切割后的点云文件夹
    const std::string out_dir;

    // 输出的POSE文件
    const std::string out_pose_file;

    // 输出的切割后的时间戳文件
    const std::string out_stamp_file;

    // 
    bool is_set_stamp;

    // 
    uint64_t start_stamp;

    // 
    uint64_t end_stamp;

    split_params(const std::string& pcd_dir, 
        const std::string& base_rot_file, 
        const std::string& frame_rot_file,
        const std::string& out_dir,
        const std::string& out_pose_file,
        const std::string& out_stamp_file
        )
        :pcd_dir(pcd_dir),
        base_rot_file(base_rot_file),
        frame_rot_file(frame_rot_file),
        out_dir(out_dir),
        out_pose_file(out_pose_file),
        out_stamp_file(out_stamp_file)
    {}
};

struct blend_params
{
    // 基准的切割后点云文件夹
    const std::string base_dir;

    // 待配准的切割后点云文件夹
    const std::string frame_dir;

    // 切割产生的时间戳文件
    const std::string stamp_file;

    // 输出的POSE文件
    const std::string out_pose_file;

    // 输入的POSE文件
    const std::string in_pose_file;

    // 最后输出的点云文件夹
    const std::string cloud_dir;

    // 是否同向路的拼合
    const bool is_same_direction;

    // ICP阈值
    const double min_icp_threshold;

    const int cache_frame_num;

    blend_params(
        const std::string& base_dir,
        const std::string& frame_dir,
        const std::string& stamp_file,
        const std::string& in_pose_file,
        const std::string& cloud_dir,
        const std::string& out_pose_file,
        const bool is_same_direction,
        const double min_icp_threshold,
        const int cache_frame_num)
        : base_dir(base_dir),
        frame_dir(frame_dir),
        stamp_file(stamp_file),
        in_pose_file(in_pose_file),
        out_pose_file(out_pose_file),
        cloud_dir(cloud_dir),
        is_same_direction(is_same_direction),
        min_icp_threshold(min_icp_threshold),
        cache_frame_num(cache_frame_num)
    {}
};

struct convert_params
{
    const std::string in_pose_log;
    const std::string in_start_pcd;
    const std::string in_end_pcd;
    const std::string out_pcd;
    const std::string center_point;
    convert_params(const std::string& in_pose_log,
        const std::string& in_start_pcd,
        const std::string& in_end_pcd,
        const std::string& out_pcd,
        const std::string& center_point)
        : in_pose_log(in_pose_log),
        in_start_pcd(in_start_pcd),
        in_end_pcd(in_end_pcd),
        out_pcd(out_pcd),
        center_point(center_point)
    {}
};

struct closure_params
{
    const std::string in_origin_pose_file;
    const std::string in_pcd_dir;
    const std::string out_base_split_dir;
    const std::string out_frame_split_dir;
    const std::string out_cloud_dir;
    const std::string out_new_pose_file;
    closure_params(
        const std::string &in_origin_pose_file,
        const std::string &in_pcd_dir,
        const std::string &out_base_split_dir,
        const std::string &out_frame_split_dir,
        const std::string &out_cloud_dir,
        const std::string &out_new_pose_file)
        : in_origin_pose_file(in_origin_pose_file),
        in_pcd_dir(in_pcd_dir),
        out_base_split_dir(out_base_split_dir),
        out_frame_split_dir(out_frame_split_dir),
        out_cloud_dir(out_cloud_dir),
        out_new_pose_file(out_new_pose_file)
    {}
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END