#pragma once
#include "macros.h"
#include "point_type.h"
#include "point_cloud.h"

/*
 *  功能描述: 定义了一些点云工程通用的类型
 */
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

typedef PointXYZRGBA CloudItem;//XYZRGBA格式的点云单元
typedef PointCloud<CloudItem> Cloud;//由XYZRGBA格式的点云单元构成的点云数据
typedef Cloud::ConstPtr CloudConstPtr;//指向带有颜色信息的点云的常量指针
typedef Cloud::Ptr CloudPtr;//指向带有颜色信息的点云的指针,点云pcl中Ptr为shared_ptr类型

typedef PointXYZRGBA_F CloudItem_F;//XYZRGBA格式的点云单元
typedef PointCloud<CloudItem_F> Cloud_F;//由XYZRGBA格式的点云单元构成的点云数据
typedef Cloud_F::ConstPtr Cloud_FConstPtr;//指向带有颜色信息的点云的常量指针
typedef Cloud_F::Ptr Cloud_FPtr;//指向带有颜色信息的点云的指针,点云pcl中Ptr为shared_ptr类型



CLOUD_BLEND_DOUBLE_NAMESPACE_END