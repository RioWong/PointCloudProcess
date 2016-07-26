#pragma once
#include "cmm_types.h"
#include "params.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN

struct POINTXYZd
{
    double x,y,z;
};
class PointCloudSpliter
{
public:
    PointCloudSpliter(void);
    ~PointCloudSpliter(void);
    static int main(const split_params & params);
    static POINTXYZd base_retification(std::vector<CloudStampRot> &base_rot);
    static void uncentralization(CloudPtr cloud,const CloudStampRot& stamp_rot,POINTXYZd centra_pt);      //中心化并减去统一的中心
    static void split_pcd_by_log(
        const std::string& pcd_dir, 
        std::string out_slice_path,
        std::string out_stamp_path,
        std::vector<CloudStampRot>& stamp_rots,
        POINTXYZd centra__point,
        const uint64_t begin_stamp,
        const uint64_t end_stamp);


private:
    static double get_distance(POINTXYZd startpoint,POINTXYZd endpoint);
    static POINTXYZd get_centra_point(std::vector<CloudStampRot>& stamp_rots);
    static void WGS84ToUTM(double& x,double& y,std::string proj_param);  //wgs84转utm坐标，param:经度，纬度，投影带中央经度
    static void UTMToUTM2(double& x,double& y,std::string proj_param,std::string proj_param2);  //utm转wgs84坐标，param:utm坐标，投影带中央经度
    static void UTMToWGS84(double& x,double& y,std::string proj_param);  //utm转wgs84坐标，param:utm坐标，投影带中央经度
    static POINTXYZd get_centra_point(const CloudStampRot& stamp_rot,POINTXYZd centra_pt);   //取得中心点
    static void out_put_ply(std::vector<CloudPtr>& clouds,uint64_t &index,std::string out_slice_path,POINTXYZd centra_point);      //滤波并输出，带时间戳
};
CLOUD_BLEND_DOUBLE_NAMESPACE_END
