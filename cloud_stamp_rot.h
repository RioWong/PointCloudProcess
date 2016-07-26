#pragma once
#include <Eigen/Eigen>
#include <boost/unordered_map.hpp>
#include "cmm_types.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
class CloudStampRot
{
public:
    CloudStampRot();

    CloudStampRot(bool is_valid)
    {
        _is_valid = false;
    }

    CloudStampRot(uint64_t stamp, Eigen::Matrix4d rot, CloudPtr cloud_line, double value_icp)
        : _is_valid(true), _cloud_line_src(new Cloud())
    {
        _stamp = stamp;
        _rot = rot;
        _cloud_line = cloud_line;
        _value_icp = value_icp;
        *_cloud_line_src = *cloud_line;
    }

    ~CloudStampRot(void);
    uint64_t _stamp;
    Eigen::Matrix4d _rot;
    //Eigen::Matrix4d _rotd;
    CloudPtr _cloud_line;
    CloudPtr _cloud_line_src;
    bool _is_valid;
    double _value_icp;
    double _time_stamp_d;
    std::string _time_stamp_str;
private:
    //DISALLOW_COPY_AND_ASSIGN(CloudStampRot);
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END