/*
*   功能描述: 点云全局cache
*            用于保存全局的点云,和根据当前位置获取特定范围内的点云数据
*/
#pragma once
#include <boost/unordered_map.hpp>
#include <list>
#include <Eigen/Eigen>
#include "macros.h"
#include "cmm_types.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
typedef unsigned long long MapKeyType;

class CloudGridItem
{
public:
    Cloud nodepoint;
    int x;
    int y;

    CloudGridItem()
        : x(0), y(0)
    {
    }

    bool cmp_pos(int xTmp, int yTmp)
    {
        if (x == xTmp && y == yTmp) {
            return true;
        } else
            return false;
    }
};


class CloudGrid
{
    SINGLETON_CLASS(CloudGrid)

public:
    CloudGrid(bool flag)
        : _idx(0),
        _is_first_call_add(true),
        _hash_key_base_x(0),
        _hash_key_base_y(0)
    {
    }

    void qframe_clear();
    void add_cloud(CloudPtr newpoint);
    void get_cloud_with_pos(CloudPtr& CloudCache, Eigen::Matrix4f curRot);
    void get_cloud_with_pos(CloudPtr& CloudCache, Eigen::Matrix4f curRot, int dis);
    void get_cloud_with_pos_short(CloudPtr& CloudCache, Eigen::Matrix4f curRot, int num_short);
    void get_cloud_with_pos(CloudPtr& cloud_cache, CloudItem min_xyz, CloudItem max_xyz);
    void get_grid_cloud(CloudPtr& Cloud);
    void get_grid_cloud(CloudPtr src_cloud, CloudPtr src_cloud_out, CloudPtr dst_cloud, float dis_threshold);
    void clear();
    void add_cloud_internal(CloudPtr newpoint);
    static const int MAX_DIS = 60;
private:
    int _hash_key_base_x;
    int _hash_key_base_y;
    bool _is_first_call_add;
    std::list<CloudPtr> _qframe;
    boost::unordered_map<MapKeyType, CloudGridItem> _grid;
    uint16_t _idx;

    MapKeyType get_hash_key(int x, int y) const
    {
        MapKeyType key = (((MapKeyType) (x - _hash_key_base_x)) << 32) + (MapKeyType) (y - _hash_key_base_y);
        return key;
    }

    float dis_two_point(CloudItem point1, CloudItem point2)
    {
        return (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) *
            (point1.y - point2.y) + (point1.z - point2.z) * (point1.z - point2.z);
    }

    bool is_2point_high_x(CloudItem point1, CloudItem point2, double x)
    {
        return (std::abs(point1.x - point2.x) > x || std::abs(point1.y - point2.y) > x ||
            std::abs(point1.z - point2.z) > x);
    }

    DISALLOW_COPY_AND_ASSIGN(CloudGrid);
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END