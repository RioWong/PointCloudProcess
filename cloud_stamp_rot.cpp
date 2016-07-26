#include "stdafx.h"
#include "cloud_stamp_rot.h"


CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
CloudStampRot::CloudStampRot(void)
    : _stamp(0), _rot(Eigen::Matrix4d::Zero()), _cloud_line(new Cloud), _is_valid(true),
      _value_icp(-2), _cloud_line_src(new Cloud()), _time_stamp_d(0)//, _rotd(Eigen::Matrix4d::Zero())
{
}

CloudStampRot::~CloudStampRot(void)
{
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END