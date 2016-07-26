#pragma once
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include "macros.h"
#include "cloud_stamp_rot.h"
#include "params.h"
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

class PointCloudClosure
{
public:
    PointCloudClosure(void);
    ~PointCloudClosure(void);
    static int main(const closure_params &params);
    struct same_segment
    {
        uint64_t base_start_stamp;
        uint64_t base_end_stamp;
        uint64_t frame_start_stamp;
        uint64_t frame_end_stamp;
    };
private:
    static double distance_sqr_rot(const Eigen::Matrix4d &rot1, const Eigen::Matrix4d &rot2);
    static uint64_t minus_abs(uint64_t a, uint64_t b);
    static void get_overlap_stamp(const std::vector<CloudStampRot> &rots, std::vector<same_segment> &overlap_segs);
    static int get_index_from_rots(const std::vector<CloudStampRot> &rots, uint64_t stamp);
    static bool do_lum_elch(std::vector<CloudStampRot> &rots, int start_index, int end_index, const Eigen::Matrix4d &loop_transform);
    static bool do_loop_closure(std::vector<CloudStampRot> &ori_rots, const std::vector<CloudStampRot> &opt_rots);
    static bool cleanup_pcd_in_dir(boost::filesystem::path dir);
};


CLOUD_BLEND_DOUBLE_NAMESPACE_END