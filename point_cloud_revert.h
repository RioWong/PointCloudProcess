/*根据pos文件恢复点云*/
#pragma once
#include "cmm_types.h"
#include "params.h"
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

class PointCloudRevert
{
public:
    PointCloudRevert(void);
    ~PointCloudRevert(void);
    static int main(const convert_params& params);
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END
