#include "stdafx.h"
#include "point_cloud_revert.h"
#include "file_helper.h"
#include "string_helper.h"
#include "cloud_stamp_rot.h"
#include "point_cloud_spliter.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;

PointCloudRevert::PointCloudRevert(void)
{
}
PointCloudRevert::~PointCloudRevert(void)
{
}
int PointCloudRevert::main(const convert_params& params)
{
    const string& pos_log_path = params.in_pose_log;
    const string& start_pcd_path = params.in_start_pcd;
    const string& end_pcd_path = params.in_end_pcd;
    const string& output_pcd_path = params.out_pcd;
    string center_point = params.center_point;
    vector<string> point_vec = StringHelper::split(center_point, ",");
    if (point_vec.size() != 3) {
        cout << "center point error" << endl;
        return -1;
    }
    POINTXYZd centra_pt;
    centra_pt.x = boost::lexical_cast<double>(boost::trim_copy(point_vec[0]));
    centra_pt.y = boost::lexical_cast<double>(boost::trim_copy(point_vec[1]));
    centra_pt.z = boost::lexical_cast<double>(boost::trim_copy(point_vec[2]));
    vector<CLOUD_BLEND_DOUBLE_NAMESPACE::CloudStampRot> rots;
    //投影参数
    string proj_str;
    CLOUD_BLEND_DOUBLE_NAMESPACE::FileHelper::load_file_rot_icp_hdmap(pos_log_path, rots, proj_str);
    //POINTXYZd centra_pt = CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudSpliter::base_retification(rots);
    for (int i = 0; i < rots.size(); i++) //修正待配准的平移向量
    {
        rots[i]._rot.block<3, 1>(0, 3) -= Eigen::Vector3d(centra_pt.x, centra_pt.y, centra_pt.z);
    }
    boost::filesystem::path path_end(end_pcd_path);
    string filename_end = path_end.filename().string();
    boost::filesystem::path path_start(start_pcd_path);
    string filename_start = path_start.filename().string();
    std::vector<std::string> res1, res2;
    boost::split(res1, filename_start, boost::is_any_of("."));
    boost::split(res2, filename_end, boost::is_any_of("."));
    if ((res1.size() != 2) || (res2.size() != 2)) {
        return false;
    }
    uint64_t start_index, end_index;
    start_index = boost::lexical_cast<uint64_t>(res1[0]);
    end_index = boost::lexical_cast<uint64_t>(res2[0]);
    Cloud_FPtr cloud_all(new Cloud_F);
    for (uint64_t n = start_index; n < end_index + 1; n++) {
        stringstream pcd_path;
        pcd_path << n << ".pcd";
        Cloud_FPtr cloud(new Cloud_F);
        CLOUD_BLEND_DOUBLE_NAMESPACE::io::loadPCDFile((path_start.parent_path() / pcd_path.str()).string(), *cloud);
        bool is_find = false;
        for (int i = 0; i < rots.size(); i++) {
            if (rots[i]._stamp == n) {
                is_find = true;
                for (int j = 0; j < cloud->size(); j++) {
                    cloud->points[j].getVector4fMap() = rots[i]._rot.cast<float>() * cloud->points[j].getVector4fMap();
                }
                break;
            }
        }
        if (is_find) {
            *cloud_all += *cloud;
        } else {
            cout << "stamp not found, skip " << n << endl;
        }
    }
    CLOUD_BLEND_DOUBLE_NAMESPACE::io::savePCDFileBinary(output_pcd_path, *cloud_all);
    return 0;
}
CLOUD_BLEND_DOUBLE_NAMESPACE_END
