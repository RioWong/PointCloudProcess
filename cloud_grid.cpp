#include "stdafx.h"
#include "cloud_grid.h"
#include "point_cloud_helper.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::set;
using std::cout;
using std::endl;
using Eigen::Matrix4f;
using boost::unordered_map;

const double MAX_DIS_2POINT_X = 0.04;
const double MAX_DIS_2POINT = MAX_DIS_2POINT_X* MAX_DIS_2POINT_X; //小于0.15米距离的合成为1个点
const double INIT_MAP_SIZE = 700000.0; //map初始大小
const int GRID_DIS = 10;
//const int MAX_DIS = 40; //根据位置获取周围40米内的点云数据,组成新的点云
const bool is_rebuild_cache = true;

CloudGrid::CloudGrid()
        : _idx(0),
        _is_first_call_add(true),
        _hash_key_base_x(0),
        _hash_key_base_y(0)
{
    //全局存储的点云量比较大,避免反复扩充大小影响性能.
    _grid.rehash(INIT_MAP_SIZE);
}

void CloudGrid::qframe_clear()
{
    _qframe.clear();
}

void CloudGrid::add_cloud_internal(CloudPtr newpoint)
{
	//PointCloudHelper::remove_duplicate(newpoint,MAX_DIS_2POINT_X,10);
//#pragma omp parallel for
    for (int i = 0; i < newpoint->size(); i++) {
        int irow = newpoint->points[i].x;
        int icol = newpoint->points[i].y;
        bool comparev = false;

        if (_is_first_call_add) {
            _hash_key_base_x = irow;
            _hash_key_base_y = icol;
            _is_first_call_add = false;
        }

        MapKeyType key = get_hash_key(irow, icol);
        //简化不考虑边界
        unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
        if (it == _grid.end()) {
            CloudGridItem tempnote;
            tempnote.x = irow;
            tempnote.y = icol;
            tempnote.nodepoint.push_back(newpoint->points[i]);
            _grid[key] = tempnote;
        } else {
            comparev = true;
            bool mindis = true;
            for (int k = 0; k < it->second.nodepoint.size(); k++) {
                if (is_2point_high_x(it->second.nodepoint[k], newpoint->points[i],
                    MAX_DIS_2POINT_X)) {
                    continue;
                }
                //距离小于MAX_DIS_2_global_transformPOINT开方的合并为1个点,这里需精确,不能优化,后续要用到
                if (dis_two_point(it->second.nodepoint[k], newpoint->points[i]) <
                    MAX_DIS_2POINT) {
                    mindis = false;
                    break;
                }
            }
            if (mindis) {
                it->second.nodepoint.push_back(newpoint->points[i]);
            }
        }
    }
}

void CloudGrid::get_cloud_with_pos(CloudPtr& cloud_cache, Matrix4f curRot)
{
    get_cloud_with_pos(cloud_cache,curRot,MAX_DIS);
}

void CloudGrid::get_cloud_with_pos(CloudPtr& cloud_cache, Matrix4f curRot ,int dis){
    _idx++;
    //if (_idx % 5 != 1) {
    //    return;
    //}
    int irow = curRot(0, 3);
    int icol = curRot(1, 3);
    if (cloud_cache->size() != 0) {
        cloud_cache->clear();
    }
    //基于当前位置取特定区块内点云
    for (int i = irow - dis; i < irow + dis; i++) {
        for (int j = icol - dis; j < icol + dis; j++) {
            MapKeyType key = get_hash_key(i, j);
            unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
            if (it == _grid.end()) {
            } else {
                for (int k = 0; k < it->second.nodepoint.size(); k++) {
                    cloud_cache->points.push_back(it->second.nodepoint[k]);
                }
            }
        }
    }
}

void CloudGrid::get_cloud_with_pos(CloudPtr& cloud_cache, CloudItem min_xyz,CloudItem max_xyz){
	_idx++;
	//if (_idx % 5 != 1) {
	//    return;
	//}
	if (cloud_cache->size() != 0) {
		cloud_cache->clear();
	}
	//基于当前位置取特定区块内点云
	for (int i = min_xyz.x; i < max_xyz.x; i++) {
		for (int j =min_xyz.y; j < max_xyz.y; j++) {
			MapKeyType key = get_hash_key(i, j);
			unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
			if (it == _grid.end()) {
			} else {
				for (int k = 0; k < it->second.nodepoint.size(); k++) {
					cloud_cache->points.push_back(it->second.nodepoint[k]);
				}
			}
		}
	}
}
void CloudGrid::get_cloud_with_pos_short(CloudPtr& cloud_cache, Matrix4f cur_rot, int num_short){
	int frame_size = 200 + num_short;
	if(_qframe.size() > frame_size)
	{
		cout << "get cloud short .... "<< frame_size <<endl;
		cloud_cache->clear();
		int idx=0;
		std::list<CloudPtr>::iterator it = _qframe.end();
		it--;
		for (; it != _qframe.begin() && idx < frame_size; -- it,idx++)  
		{  
			CloudPtr& item=*it;
			*cloud_cache += *item;  
		}  
	}else{
		get_cloud_with_pos(cloud_cache,cur_rot);
	}
}
void CloudGrid::get_grid_cloud(CloudPtr& cloud){
    cloud->clear();
    for (unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.begin();
        it != _grid.end(); it++) {
        for (int i = 0; i < it->second.nodepoint.size(); i++) {
            cloud->push_back(it->second.nodepoint[i]);
        }
    }
}

void CloudGrid::get_grid_cloud(CloudPtr src_cloud, CloudPtr src_cloud_out, CloudPtr dst_cloud, float dis_threshold)
{
	//src_cloud_out->clear();
	dst_cloud->clear();
	if (dst_cloud->size() != 0) {
		dst_cloud->clear();
	}
	CloudPtr src_cloud_out_temp(new Cloud());
	unordered_map<MapKeyType, set<int> > grid_index_map;
	src_cloud_out_temp->header.stamp = src_cloud->header.stamp;
	for (int i = 0; i < src_cloud->size(); i++) {
		int irow = src_cloud->points[i].x;
		int icol = src_cloud->points[i].y;

		MapKeyType key = get_hash_key(irow, icol);
		//简化不考虑边界
		unordered_map< MapKeyType, CloudGridItem>::iterator it = _grid.find(key);
		if (it == _grid.end()) {
			continue;
		}
		unordered_map<MapKeyType, set<int> >::iterator map_it = grid_index_map.find(key);
		bool is_find = false;
		for (int k = 0; k < it->second.nodepoint.size(); k++) {
			if (is_2point_high_x(it->second.nodepoint[k], src_cloud->points[i],
				dis_threshold)) {
					continue;
			}
			is_find = true;
			if (map_it != grid_index_map.end() && map_it->second.find(k) != map_it->second.end()) {
				continue;
			}
			if (map_it != grid_index_map.end()) {
				map_it->second.insert(k);
			} else {
				grid_index_map[key].insert(k);
			}
			dst_cloud->points.push_back(it->second.nodepoint[k]);
		}
		if (is_find) {
			src_cloud_out_temp->points.push_back(src_cloud->points[i]);
		}
	}
	*src_cloud_out = *src_cloud_out_temp;
}

void CloudGrid::clear(){
    _grid.clear();
    cout << "clear cache ..." << endl;
    _is_first_call_add = true;
    _hash_key_base_x = 0;
    _hash_key_base_y = 0;
}
} // cloud_icp_reg