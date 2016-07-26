#include "stdafx.h"
#include "process_status.h"


CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
ProcessStatus::ProcessStatus(void)
    : _pcap_file(""), _global_transform(Eigen::Matrix4d::Identity()), _frame_count_total(0),
      _frame_count_process(0), _skip_frame(0), _start_frame(0),
      _is_write_stamp(true), _last_mt(Eigen::Matrix4f::Identity()), _cloud_src(new Cloud),
      _cloud_result(new Cloud), _cloud_cache(new Cloud), _is_batch_mode(false),
      _cloud_temp_src_last(new Cloud), _stamp_rot_unvalid(false), min_icp_threshold(-1)
{
}

ProcessStatus::~ProcessStatus(void)
{
}

CloudStampRot& ProcessStatus::get_cloud_rot_with_stamp(uint64_t stamp)
{
    for (int i = 0; i < _clouds_stamp_rot_line.size(); i++) {
        if (_clouds_stamp_rot_line[i]._stamp == stamp) {
            return _clouds_stamp_rot_line[i];
        }
    }
    return _stamp_rot_unvalid;
}

void ProcessStatus::clear_cloud_line_cache(int num)
{
    if (_clouds_stamp_rot_line.size() < num) {
        return;
    }
    //ÇåÀíline»º³å
    for (int i = 0; i < _clouds_stamp_rot_line.size() - num; i++) {
        if (i >= _clouds_stamp_rot_line.size()) {
            break;
        }
        _clouds_stamp_rot_line[i]._cloud_line->clear();
        _clouds_stamp_rot_line[i]._cloud_line_src->clear();
    }
}

int ProcessStatus::get_cloud_rot_index_with_stamp(uint64_t stamp)
{
    int index = -1;
    for (int i = 0; i < _clouds_stamp_rot_line.size(); i++) {
        if (_clouds_stamp_rot_line[i]._stamp == stamp) {
            index = i;
            break;
        }
    }
    return index;
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END