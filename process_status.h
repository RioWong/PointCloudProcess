#pragma once
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>
#include <vector>
#include "cmm_types.h"
#include "cloud_stamp_rot.h"
#include "macros.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
typedef enum
{
    ENUM_MODE_PROCESS_POS = 0,
    ENUM_MODE_PROCESS_LOOP,
    ENUM_MODE_PROCESS_LINE,
    ENUM_MODE_PROCESS_LOOP_TRACE,
    ENUM_MODE_PROCESS_LOOP_TRACE_NEW,
    ENUM_MODE_PROCESS_OUT_PUT = 9,
    ENUM_MODE_PROCESS_FIXPOS,
} enumModeProcess;

class ProcessStatus
{
public:
    ProcessStatus(void);
    ~ProcessStatus(void);
    CloudStampRot& get_cloud_rot_with_stamp(uint64_t stamp);
    void clear_cloud_line_cache(int num);
    // 根据时间戳找到对应的index
    int get_cloud_rot_index_with_stamp(uint64_t stamp);
    std::string _pcap_file;
    bool _is_batch_mode;
    bool _is_debug_mode;
    bool _is_step_mode;
    Eigen::Matrix4d _global_transform;
    int _frame_count_total; //所有处理的帧数,包含跳过的帧数
    int _frame_count_process; //处理的帧数
    int _skip_frame;
    uint64_t _start_frame;
    //是否写入时间戳
    bool _is_write_stamp;
    Eigen::Matrix4f _last_mt;

    //存放loop用的点云集合
    //vector< CloudPtr> _clouds_line;
    //boost::unordered_map< int, Eigen::Matrix4f> _map_clouds_rot;
    //boost::unordered_map< int, int64_t> _map_clouds_stamp;
    std::vector<CloudStampRot> _clouds_stamp_rot_line;
    //vector< uint64_t> _map_clouds_stamps;
    //vector< Eigen::Matrix4f> _optimize_rot;
    enumModeProcess _current_mode;
    std::map<uint64_t, uint64_t> _ignore_stamp;
    //vector< pair< pair<uint64_t, uint64_t>, Eigen::Matrix4f> > _map_loops;
    //vector< pair<uint64_t, uint64_t> > _map_ignore_loops;
    //点云数据
    CloudPtr _cloud_src; //存放上一帧数据
    CloudPtr _cloud_result; //存放结果数据
    CloudPtr _cloud_cache;

    int _num_start_frame_cache; // 缓冲帧数
    bool do_split_icp; //是否做大块ICP之后再做小块的ICP

    CloudPtr _cloud_temp_src_last; //存放上一帧的原始数据

    //EnumPtDev _current_dev;
    std::set<uint64_t> _ignore_invalid_stamp;

    double min_icp_threshold; //建立场景时,设置最小允许通过的icp阈值
private:
    DISALLOW_COPY_AND_ASSIGN(ProcessStatus);
    CloudStampRot _stamp_rot_unvalid;

    //boost::unordered_map< uint64_t,Eigen::Matrix4f&> _map_clouds_rot_stamps;
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END
