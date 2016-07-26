// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include <stdio.h>


// TODO: 在此处引用程序需要的其他头文件
#include <iostream>
#include <limits>
#include <iomanip>
#include <string>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <time.h>

#include <boost/smart_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <opencv/cv.h>

#include <proj_api.h>

#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "ICP.h"

#include "cmm_types.h"
#include "point_type.h"
#include "point_cloud.h"
#include "voxel_grid.h"
#include "kd_tree.h"
#include "channel_properties.h"
#include "conversions.h"
#include "file_io_helper.h"
#include "for_each_type.h"
#include "lzf.h"
#include "pcd_helper.h"
#include "point_traits.h"
#include "sensor_image.h"
#include "sensor_point_cloud2.h"
#include "sensor_point_field.h"

#include "static.h"

#include "cloud_stamp_rot.h"
#include "file_helper.h"
#include "point_cloud_helper.h"
#include "process_status.h"
#include "point_cloud_revert.h"

