#include "stdafx.h"
#include "point_cloud_spliter.h"
#include "cloud_grid.h"
#include "file_helper.h"
#include "string_helper.h"
#include "point_cloud_helper.h"
#include "params.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
using std::cout;
using std::endl;
using std::list;
using std::vector;
using std::string;
using std::ofstream;
using Eigen::Matrix4f;
using Eigen::Matrix3f;
using Eigen::Vector3d;
using boost::unordered_map;
using std::stringstream;
PointCloudSpliter::PointCloudSpliter(void)
{
}


PointCloudSpliter::~PointCloudSpliter(void)
{
}

double PointCloudSpliter::get_distance(POINTXYZd startpoint, POINTXYZd endpoint) //求三维两点间距离
{
    double _distance = (startpoint.x - endpoint.x) * (startpoint.x - endpoint.x) + (startpoint.y - endpoint.y) * (startpoint.y - endpoint.y) + (startpoint.z - endpoint.z) * (startpoint.z - endpoint.z);
    return sqrt(_distance);
}

POINTXYZd PointCloudSpliter::get_centra_point(vector<CloudStampRot>& stamp_rots) //取得中心点,所有pcd的中心点
{
    double x, y, z;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    for (int i = 0; i < stamp_rots.size(); i++) {
        x = x * double(i) / (i + 1) + stamp_rots[i]._rot(0, 3) / double(i + 1);
        y = y * double(i) / (i + 1) + stamp_rots[i]._rot(1, 3) / double(i + 1);
        z = z * double(i) / (i + 1) + stamp_rots[i]._rot(2, 3) / double(i + 1);
    }
    POINTXYZd centra_point;
    centra_point.x = x;
    centra_point.y = y;
    centra_point.z = z;
    return centra_point;
}

void PointCloudSpliter::WGS84ToUTM(double& x, double& y, std::string proj_param) //wgs84转utm坐标，param:经度，纬度，投影带中央经度
{
    projPJ pj_merc, pj_latlong;
    if (!(pj_merc = pj_init_plus(proj_param.c_str())))
        exit(1);
    if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")))
        exit(1);
    cout << x << " " << y << endl;
    x *= DEG_TO_RAD;
    y *= DEG_TO_RAD;


    pj_transform(pj_latlong, pj_merc, 1, 1, &x, &y, NULL);
}

void PointCloudSpliter::UTMToUTM2(double& x, double& y, std::string proj_param, std::string proj_param2) //utm转wgs84坐标，param:utm坐标，投影带中央经度
{
    projPJ pj_merc, pj_merc2;
    if (!(pj_merc = pj_init_plus(proj_param.c_str())))
        exit(1);
    if (!(pj_merc2 = pj_init_plus(proj_param2.c_str())))
        exit(1);

    pj_transform(pj_merc, pj_merc2, 1, 1, &x, &y, NULL);
}

void PointCloudSpliter::UTMToWGS84(double& x, double& y, std::string proj_param) //utm转wgs84坐标，param:utm坐标，投影带中央经度
{
    projPJ pj_merc, pj_latlong;
    if (!(pj_merc = pj_init_plus(proj_param.c_str())))
        exit(1);
    if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")))
        exit(1);
    pj_transform(pj_merc, pj_latlong, 1, 1, &x, &y, NULL);

    x /= DEG_TO_RAD;
    y /= DEG_TO_RAD;
}

POINTXYZd PointCloudSpliter::get_centra_point(const CloudStampRot& stamp_rot, POINTXYZd centra_pt) //取得中心点
{
    double x, y, z;
    x = stamp_rot._rot(0, 3);
    y = stamp_rot._rot(1, 3);
    z = stamp_rot._rot(2, 3);
    POINTXYZd centra_point;
    centra_point.x = x;
    centra_point.y = y;
    centra_point.z = z;
    centra_point.x -= centra_pt.x;
    centra_point.y -= centra_pt.y;
    centra_point.z -= centra_pt.z;
    return centra_point;
}

void PointCloudSpliter::uncentralization(CloudPtr cloud, const CloudStampRot& stamp_rot, POINTXYZd centra_pt) //中心化并减去统一的中心
{
    Eigen::Matrix4d mat = stamp_rot._rot;
    mat.block<3, 1>(0, 3) -= Eigen::Vector3d(centra_pt.x, centra_pt.y, centra_pt.z);
    CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudHelper::transformPointCloud(*cloud, *cloud, mat);
}

void PointCloudSpliter::out_put_ply(vector<CloudPtr>& clouds, uint64_t& index, std::string out_slice_path,POINTXYZd centra_pt) //滤波并输出，带时间戳
{
    CloudPtr src_cloud(new Cloud);
    CloudPtr dst_cloud(new Cloud);
    for (int i = 0; i < clouds.size(); i++) //给点打上时间戳0~
    {
        for (int j = 0; j < clouds[i]->points.size(); j++) {
            clouds[i]->points[j].stamp_id = uint32_t(i);
        }
        *src_cloud += *clouds[i];
    }
    CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudHelper::remove_duplicate(src_cloud, 0.04);
    CLOUD_BLEND_DOUBLE_NAMESPACE::tree_filter(src_cloud, dst_cloud);
    vector<CloudPtr> output_clouds;
    for (int i = 0; i < clouds.size(); i++) {
        CloudPtr tempcloud(new Cloud);
        output_clouds.push_back(tempcloud);
    }
    for (int i = 0; i < dst_cloud->points.size(); i++) {
        uint32_t stamp = dst_cloud->points[i].stamp_id;
        output_clouds[stamp]->points.push_back(dst_cloud->points[i]);
    }
    /*#pragma omp parallel for*/
    for (int i = 0; i < output_clouds.size(); i++) {
        stringstream out;
        out << index << ".pcd";
        string filename = out.str();
        string str = (boost::filesystem::path(out_slice_path) / filename).string();
//         Cloud_FPtr float_out(new Cloud_F);
//         float_out->resize(output_clouds[i]->size());
        for (int j = 0; j < output_clouds[i]->size(); j++) {
            output_clouds[i]->points[j].x+=centra_pt.x;;
            output_clouds[i]->points[j].y+=centra_pt.y;
            output_clouds[i]->points[j].z+=centra_pt.z;

        }
        output_clouds[i]->header.stamp=index;
		if (output_clouds[i]->size()<=0)
		{
			cout<<"warning: point size in pcd empty " <<str<<endl;
		}
		else
		{
			CLOUD_BLEND_DOUBLE_NAMESPACE::io::savePCDFileBinary(str, *output_clouds[i]);
		}

        //CLOUD_BLEND_DOUBLE_NAMESPACE::io::savePCDFileBinary(str, *float_out);
        cout << output_clouds[i]->points.size() << " " << str << endl;
        index++;
    }
}

void PointCloudSpliter::split_pcd_by_log(
    const std::string& pcd_dir, 
    std::string out_slice_path, 
    std::string out_stamp_path, 
    vector<CloudStampRot>& stamp_rots, 
    POINTXYZd centra__point, 
    const uint64_t begin_stamp,
    const uint64_t end_stamp)
{
    cout << "start spliting cloud..." << endl;
    uint64_t index = 0;
    uint64_t stamp_index=0;
    vector<CloudPtr> pca_blocks;//每10块聚成一个大块做pca
    CloudPtr temp_cloud(new Cloud);//每5m的pcd聚成一块
    POINTXYZd start_pt, end_pt;
    start_pt = get_centra_point(stamp_rots[0], centra__point); //起始点  
    ofstream ofile(out_stamp_path.c_str());
    ofile.setf(std::ios::fixed);
    //ofile << centra__point.x << "," << centra__point.y << "," << centra__point.z << endl;
    ofile << 0 << "," << 0<< "," << 0<< endl;
    ofile.precision(6);
    ofile <<stamp_index<<" ";
    for (int i = 0; i < stamp_rots.size(); i++) {
        if (stamp_rots[i]._stamp < begin_stamp || stamp_rots[i]._stamp > end_stamp) {
            continue;
        }
        end_pt = get_centra_point((stamp_rots[i]), centra__point);
        ofile << stamp_rots[i]._time_stamp_d << " ";
        Cloud_FPtr in_cloud(new Cloud_F);
        boost::filesystem::path pcd_path(pcd_dir);
        stringstream str_stamp;
        str_stamp << stamp_rots[i]._stamp << ".pcd";
        std::string pcd_file = (pcd_path / str_stamp.str()).string();
        if(CLOUD_BLEND_DOUBLE_NAMESPACE::io::loadPCDFile(pcd_file, *in_cloud))
        {
            cout<<"error: cannot find pcd files:"<<pcd_file<<endl;
            return ;
        }
        vector<int> nan_index;
        CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudHelper::removeNaNFromPointCloud(*in_cloud, *in_cloud, nan_index);
        CloudPtr temp_pcd(new Cloud);
        temp_pcd->resize(in_cloud->size());
        for (int j = 0; j < in_cloud->size(); j++) {
            temp_pcd->points[j].x = in_cloud->points[j].x;
            temp_pcd->points[j].y = in_cloud->points[j].y;
            temp_pcd->points[j].z = in_cloud->points[j].z;
            temp_pcd->points[j].r = in_cloud->points[j].r;
            temp_pcd->points[j].g = in_cloud->points[j].g;
            temp_pcd->points[j].b = in_cloud->points[j].b;
            temp_pcd->points[j].a = in_cloud->points[j].a;
        }
        uncentralization(temp_pcd, stamp_rots[i], centra__point);
        *temp_cloud += *temp_pcd;
        if (get_distance(start_pt, end_pt) > 5.0) //五米切一段
        {
            ofile << endl;
            stamp_index++;
            ofile << stamp_index<<" ";
            start_pt = end_pt;
            CloudPtr cloud(new Cloud);
            *cloud = *temp_cloud;
            pca_blocks.push_back(cloud); //存入做pca的大块内
            temp_cloud->clear();
            if (pca_blocks.size() >= 10) {
                out_put_ply(pca_blocks, index, out_slice_path,centra__point);
                pca_blocks.clear();
            }
        }
        if (i == stamp_rots.size() - 1 || stamp_rots[i]._stamp == end_stamp) //最后一块，不考虑5m与20块，直接pca后输出
        {
            ofile << endl;
            stamp_index++;
            ofile << stamp_index<<" ";
            CloudPtr cloud(new Cloud);
            *cloud = *temp_cloud;
            pca_blocks.push_back(cloud);
            temp_cloud->clear();
            out_put_ply(pca_blocks, index, out_slice_path,centra__point);
            pca_blocks.clear();
        }
    }
    ofile.close();
}

POINTXYZd PointCloudSpliter::base_retification(vector<CloudStampRot>& base_rot) //输入基准rot,计算中心点
{
    double x, y, z;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    for (int i = 0; i < base_rot.size(); i++) {
        x = x * double(i) / (i + 1) + base_rot[i]._rot(0, 3) / double(i + 1);
        y = y * double(i) / (i + 1) + base_rot[i]._rot(1, 3) / double(i + 1);
        z = z * double(i) / (i + 1) + base_rot[i]._rot(2, 3) / double(i + 1);
    }
    POINTXYZd centra_point;
    centra_point.x = x; //基准的中心点转到待配准的投影坐标系里面的坐标
    centra_point.y = y;
    centra_point.z = z;
    return centra_point;
}

int PointCloudSpliter::main(const split_params & params)
{
    std::string pcd_path = params.pcd_dir; //pcd路径
    std::string base_rot_path = params.base_rot_file; //基准log文件路径 
    std::string targ_rot_path = params.frame_rot_file; //目标log文件路径
    std::string out_path = params.out_dir; //输出路径
    std::string out_pose_path = params.out_pose_file;
    std::string out_stamp_path = params.out_stamp_file;
    boost::filesystem::path pcd_path_bf(pcd_path);
    if (!boost::filesystem::exists(pcd_path_bf))
    {
        cout<<"error in finding pcd_dir"<<endl;
        return 0;
    }
    boost::filesystem::path base_rot_path_bf(base_rot_path);
    if (!boost::filesystem::exists(base_rot_path_bf))
    {
        cout<<"error in fingding base_rot_file"<<endl;
        return 0;
    }
    boost::filesystem::path targ_rot_path_bf(targ_rot_path);
    if (!boost::filesystem::exists(targ_rot_path))
    {
        cout<<"error in fingding frame_rot_file"<<endl;
        return 0;
    }
    boost::filesystem::path out_pose_path_bf(out_pose_path);
    boost::filesystem::path out_pose_parent_path=out_pose_path_bf.parent_path();
    if (!boost::filesystem::exists(out_pose_parent_path))
    {
        cout<<"error in fingding out_pose_dir,create one"<<endl;
        if(!boost::filesystem::create_directory(out_pose_parent_path))
        {
            cout<<"cannot create out_pose_dir"<<endl;
            return 0;
        }
    }    
    boost::filesystem::path out_stamp_path_bf(out_stamp_path);
    boost::filesystem::path out_stamp_parent_path=out_stamp_path_bf.parent_path();
    if (!boost::filesystem::exists(out_stamp_parent_path))
    {
        cout<<"error in fingding out_stamp_dir,create one"<<endl;
        if(!boost::filesystem::create_directory(out_stamp_parent_path))
        {
            cout<<"cannot create out_pose_dir"<<endl;
            return 0;
        }
    }
    boost::filesystem::path out_path_bf(out_path);
    if (!boost::filesystem::exists(out_path_bf))
    {
        cout<<"error in fingding out_dir,create one"<<endl;
        if(!boost::filesystem::create_directory(out_path_bf))
        {
            cout<<"cannot create out_dir"<<endl;
            return 0;
        }
    }
    vector<CloudStampRot> base_rots;
    string base_proj, targ_proj; //基准和待配准的投影参数
    FileHelper::load_file_rot_icp_hdmap(base_rot_path, base_rots, base_proj);
    vector<CloudStampRot> targ_rots;

    FileHelper::load_file_rot_icp_hdmap(targ_rot_path, targ_rots, targ_proj);
    std::ofstream ofile(out_pose_path.c_str());
    ofile << base_proj << endl;
    ofile.close();
    for (int i = 0; i < targ_rots.size(); i++) //修正待配准的平移向量
    {
        UTMToUTM2(targ_rots[i]._rot(0, 3), targ_rots[i]._rot(1, 3), targ_proj, base_proj);
    }
    FileHelper::write_file_rot_hdmap(out_pose_path, targ_rots, true);
    cout << "rot read" << endl;
    POINTXYZd centra_point = base_retification(base_rots);
    cout << "centra point:" << centra_point.x << " , " << centra_point.y << endl;

//     centra_point.x=0.0;
//     centra_point.y=0.0;
//     centra_point.z=0.0;
    vector<CloudStampRot> stamp_rots;
    //vector<std::string> pcd_files;
    //pcd_files = FileHelper::get_all_ply_files2(pcd_path);
    //if (pcd_files.size()==0)
    //{
    //    cout<<"error: cannot find pcd files in folder:"<<pcd_path<<endl;
    //    return 0;
    //}
    for (int i=0;i <targ_rots.size();i ++)
    {
        boost::filesystem::path path_pcd(pcd_path);
        stringstream pcd_path_stream;
        stringstream str_stamp;
        str_stamp << targ_rots[i]._stamp;
        pcd_path_stream << (path_pcd/str_stamp.str()).string() << ".pcd";
        if(!boost::filesystem::exists(pcd_path_stream.str()))
        {
            cout<<"ignore: cannot find pcd files:"<<pcd_path_stream.str()<<endl;
        }else
        {
            stamp_rots.push_back(targ_rots[i]);
        }
    }
    if (stamp_rots.size()==0)
    {
        cout<<"error: cannot find pcd files in log file :"<<targ_rot_path<<endl;
        return 0;
    }

    uint64_t start_stamp = stamp_rots[0]._stamp;
    uint64_t end_stamp = stamp_rots[stamp_rots.size() - 1]._stamp;
    if (params.is_set_stamp) {
        start_stamp = params.start_stamp;
        end_stamp = params.end_stamp;
    }
    split_pcd_by_log(pcd_path, out_path, out_stamp_path, stamp_rots, centra_point, start_stamp, end_stamp);
    cout<<"cloud split finished"<<endl;
    return 1;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END