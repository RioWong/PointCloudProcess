#include "stdafx.h"
#include "static.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::vector;
using std::set;

void PointSegment(CloudPtr src_cloud, vector<PlanSegment>& Segment) //点分割
{
    double Radius = 0.15;
    double cosfaT = 0.940;
    double distanceT = 0.5;
    double radius_in_growning = 0.2;
    float Pr = 0.99;
    float epi = 0.5;
    int h_free;
    CalculateFeature Feature;
    //超级体素的属性容器
    boost::shared_array<LAS_POINT_PROPERTY> PointProperty = Feature.calculate_plan_parameter_rpca(src_cloud, Radius, Pr, epi);
    TreeExtration Tree;
    Segment = Tree.region_growning(PointProperty, src_cloud, distanceT, radius_in_growning, cosfaT);
}

//滤树
void tree_filter(CloudPtr src_cloud, CloudPtr dst_cloud)
{
    vector<PlanSegment> Segment;
    PointSegment(src_cloud, Segment);
    for (int i = 0; i < Segment.size(); i++) {
        if (Segment[i].PointID.size() > 50/*&&abs(Segment[i].normal_z)<0.9*/) {
            for (int j = 0; j < Segment[i].PointID.size(); j++) {
                dst_cloud->push_back(src_cloud->points[Segment[i].PointID[j]]);
            }
        }

    }
}

//滤地面的滤波器
void ground_filter(CloudPtr src_cloud, CloudPtr dst_cloud)
{
    vector<PlanSegment> Segment;
    PointSegment(src_cloud, Segment);
    set<int> groundpoints;
    for (int i = 0; i < Segment.size(); i++) //存地面点
    {
        if (std::abs(Segment[i].normal_z) > 0.9) //normal_z大于0.9为法向量朝上的地面点
        {
            for (int j = 0; j < Segment[i].PointID.size(); j++) {
                if (groundpoints.find(Segment[i].PointID[j]) == groundpoints.end()) {
                    groundpoints.insert(Segment[i].PointID[j]);
                }
            }
        }

    }
    for (int k = 0; k < src_cloud->points.size(); k++) {
        set<int>::iterator iter;
        iter = groundpoints.find(k);
        if (iter == groundpoints.end()) {
            dst_cloud->push_back(src_cloud->points[k]);
        }
    }
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END