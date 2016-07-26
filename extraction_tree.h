#pragma once

#include "data_struct.h"
#include <vector>
#include "calculate_feature.h"
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
class TreeExtration
{
public:
    std::vector<PlanSegment> region_growning(boost::shared_array<LAS_POINT_PROPERTY> point_property, CloudPtr& cloud, double distanceT, double radius, double cosfaT);
    /*就算两个方向的夹角;*/
    float compute_included_angle_between_vector(float vx1, float vy1, float vz1, float vx2, float vy2, float vz2);
    /*a_point是平面外一点,a,b,c,d是平面方程的系数;*/
    float compute_distance_from_point_to_plane(CloudItem* a_point, float a, float b, float c, float d);

    void refine_planes(std::vector<PlanSegment>& planes, const CloudPtr& cloud);
protected:
private:
    void count_pca_value(PlanSegment& plane, const CloudPtr& cloud);
    bool judge_two_planes_coplane(PlanSegment& p1, PlanSegment& p2, const CloudPtr& cloud);
    void merge_plane(PlanSegment& p1, PlanSegment& p2, const CloudPtr& cloud);
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END