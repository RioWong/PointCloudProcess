#pragma once
#include "macros.h"
#include "point_type.h"
#include "point_cloud.h"

/*
 *  ��������: ������һЩ���ƹ���ͨ�õ�����
 */
CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

typedef PointXYZRGBA CloudItem;//XYZRGBA��ʽ�ĵ��Ƶ�Ԫ
typedef PointCloud<CloudItem> Cloud;//��XYZRGBA��ʽ�ĵ��Ƶ�Ԫ���ɵĵ�������
typedef Cloud::ConstPtr CloudConstPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶĳ���ָ��
typedef Cloud::Ptr CloudPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶ�ָ��,����pcl��PtrΪshared_ptr����

typedef PointXYZRGBA_F CloudItem_F;//XYZRGBA��ʽ�ĵ��Ƶ�Ԫ
typedef PointCloud<CloudItem_F> Cloud_F;//��XYZRGBA��ʽ�ĵ��Ƶ�Ԫ���ɵĵ�������
typedef Cloud_F::ConstPtr Cloud_FConstPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶĳ���ָ��
typedef Cloud_F::Ptr Cloud_FPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶ�ָ��,����pcl��PtrΪshared_ptr����



CLOUD_BLEND_DOUBLE_NAMESPACE_END