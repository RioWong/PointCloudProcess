#pragma once

#include <boost/smart_ptr.hpp>
#include <vector>
#include <math.h>
#include "data_struct.h"
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
class CalculateFeature
{
public:
    int compute_iteration_number(float Pr, float epi, int h_free);
    float compute_min_value();
    PlanSegment calculate_plan_parameter_3points(CloudItem points[3]);
    PlanSegment calculate_plan_parameter_h_points(CloudPtr cloud);
    boost::shared_array<LAS_POINT_PROPERTY> calculate_plan_parameter(CloudPtr cloud, double radius);
    boost::shared_array<LAS_POINT_PROPERTY> calculate_plan_parameter_rpca(CloudPtr cloud, double radius, float pr, float epi);

protected:
private:
};

CLOUD_BLEND_DOUBLE_NAMESPACE_END