#include "stdafx.h"
#include "file_helper.h"
#include "string_helper.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
#define S2D(str) boost::lexical_cast<double>(boost::trim_copy(std::string(str)))

using std::ifstream;
using std::fstream;
using std::string;
using std::stringstream;
using std::cout;
using std::endl;
using std::vector;
using std::ios;
using std::map;
using std::pair;

using Eigen::Matrix4f;
using Eigen::Matrix4d;

using boost::unordered_map;

FileHelper::FileHelper(){
}

FileHelper::~FileHelper(){
}

void FileHelper::write_file(const string file_path, const string content, bool is_append)
{
    fstream outfile;
    if (is_append) {
        outfile.open(file_path.c_str(), ios::out | ios::app);
    } else {
        outfile.open(file_path.c_str(), ios::out | ios::trunc);
    }
    outfile << content << endl;
    outfile.close();
}

bool FileHelper::load_file_ignore_frame(const string pcap_file,
                                        map<uint64_t, uint64_t>& stamps)
{
    string file_path = pcap_file + "_ignore.txt";
    fstream in_file(file_path.c_str());
    if (!in_file.good()) {
        cout << "file house back empty :" << file_path << endl;
        return false;
    }
    int pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector<string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(str_line, " ");
        if (arr_strs.size() >= 2) {
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp;
            if (stamp != 0) {
                stream.clear();
                stream << arr_strs[1];
                stream >> stamps[stamp];
            }
        }
    }
    in_file.close();

    return true;
}


bool FileHelper::load_file_rot_icp(const string pcap_file, vector<CloudStampRot>& stamp_rots)
{
    //unordered_map< int, Matrix4d>& map_rots;
    //unordered_map< int, int64_t>& stamps;
    string file_path = pcap_file;

    fstream in_file(file_path.c_str());
    if (!in_file.good()) {
        cout << "file rot empty :" << file_path << endl;
        return false;
    }
    long pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector<string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    double value_icp = -2;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(str_line, " ");
        if (arr_strs.size() == 20 || arr_strs.size() == 19) {
            frame = atoi(arr_strs[0].c_str());
            Matrix4d trans = Matrix4d::Identity();
            pointcount++;
            trans << S2D(arr_strs[1]) , S2D(arr_strs[2]) ,
                S2D(arr_strs[3]) , S2D(arr_strs[4]) ,
                S2D(arr_strs[5]) , S2D(arr_strs[6]) , S2D(arr_strs[7]) ,
                S2D(arr_strs[8]) ,
                S2D(arr_strs[9]) , S2D(arr_strs[10]) , S2D(arr_strs[11]) ,
                S2D(arr_strs[12]) ,
                S2D(arr_strs[13]) , S2D(arr_strs[14]) , S2D(arr_strs[15]) ,
                S2D(arr_strs[16]);
            if (arr_strs.size() >= 19 && atoi(arr_strs[18].c_str()) == 100) {
                stringstream stream;
                stream << arr_strs[17];
                stream >> stamp;
                if (arr_strs.size() == 20) {
                    value_icp = atof(arr_strs[19].c_str());
                }
                //新数据
                //stamps[frame] = stamp;

                if (pointcount % 1000 == 0) {
                    cout << "pos载入新数据 " << stamp << endl;
                }
            } else {
                cout << "pos数据载入不正确" << str_line << endl;
                continue;
            }

            if (trans == Matrix4d::Zero()) {
                //cout << "error.........................load rot ignore......" << endl;
                //printf("error.........................load rot ignore......\n");
                //continue;
            }
            //map_rots[frame] = trans;
            CloudPtr cloud(new Cloud);
            cloud->header.stamp = stamp;
            CloudStampRot stamp_rot(cloud->header.stamp, trans, cloud, value_icp);
            stamp_rots.push_back(stamp_rot);
        } else {
            continue;
        }
    }
    in_file.close();

    return true;
}

bool FileHelper::load_file(const string pcap_file, const string suffix,
                           unordered_map<int, Matrix4d>& map_rots, unordered_map<int, int64_t>& stamps,
                           bool is_contain_stamp)
{
    string file_path;
    file_path = pcap_file + suffix;

    fstream in_file(file_path.c_str());
    if (!in_file.good()) {
        cout << "file rot empty :" << file_path << endl;
        return false;
    }
    long pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector<string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(str_line, " ");
        if (arr_strs.size() == 18 || arr_strs.size() == 17 || arr_strs.size() == 19) {
            frame = atoi(arr_strs[0].c_str());
            Matrix4d trans = Matrix4d::Identity();
            pointcount++;
            trans << S2D(arr_strs[1]) , S2D(arr_strs[2]) ,
                S2D(arr_strs[3]) , S2D(arr_strs[4]) ,
                S2D(arr_strs[5]) , S2D(arr_strs[6]) , S2D(arr_strs[7]) ,
                S2D(arr_strs[8]) ,
                S2D(arr_strs[9]) , S2D(arr_strs[10]) , S2D(arr_strs[11]) ,
                S2D(arr_strs[12]) ,
                S2D(arr_strs[13]) , S2D(arr_strs[14]) , S2D(arr_strs[15]) ,
                S2D(arr_strs[16]);
            if (arr_strs.size() == 18) {
                //旧数据
                stringstream stream;
                stream << arr_strs[17];
                stream >> stamp;
                stamp -= 100;
                stamps[frame] = stamp;
                if (pointcount % 1000 == 0) {
                    cout << "pos载入旧数据 " << stamp << endl;
                }
            } else if (arr_strs.size() == 19 && atoi(arr_strs[18].c_str()) == 100) {
                stringstream stream;
                stream << arr_strs[17];
                stream >> stamp;
                //新数据
                stamps[frame] = stamp;
                if (pointcount % 1000 == 0) {
                    cout << "pos载入新数据 " << stamp << endl;
                }
            } else {
                cout << "pos数据载入不正确" << str_line << endl;
                continue;
            }

            if (trans == Matrix4d::Zero()) {
                //cout << "error.........................load rot ignore......" << endl;
                //printf("error.........................load rot ignore......\n");
                //continue;
            }
            map_rots[frame] = trans;
        } else {
            continue;
        }
    }
    in_file.close();

    return true;
    /*
    FILE *fp = fopen(file_path.c_str(), "r");
    if (fp == NULL)
    {
    cout << "file rot empty :" << file_path;
    return false;
    }
    int frame = 0;
    float r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15;
    r0 = r1 = r2 = r3 = r4 = r5 = r6 = r7 = r8 = r9 = r10 = r11 = r12 = r13 = r14 = r15 = 0;
    while (!feof(fp))
    {
    uint64_t stamp = 0;
    if (isContainStamp)
    {
    fscanf_s(fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %lld",
    &frame, &r0, &r1, &r2, &r3, &r4, &r5, &r6, &r7, &r8, &r9, &r10, &r11, &r12, &r13, &r14, &r15, &stamp);
    }
    else
    {
    fscanf_s(fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    &frame, &r0, &r1, &r2, &r3, &r4, &r5, &r6, &r7, &r8, &r9, &r10, &r11, &r12, &r13, &r14, &r15);
    }
    Matrix4d trans = Matrix4d::Identity();
    trans << r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15;
    //cout << "load file rot :" << frame << endl <<trans<< endl << endl;
    if (trans == Matrix4d::Zero())
    {
    cout << "error.........................load rot ignore......" << endl;
    printf("error.........................load rot ignore......\n");
    continue;
    }
    map_rots[frame] = trans;
    stamps[frame] = stamp;
    //Vector4f ci;
    //Matrix42Vector4(trans, ci);
    }
    cout << "file rot load finished.rot size:" << map_rots.size() << file_path;
    fclose(fp);
    return true;
    */
}

bool FileHelper::load_file(const string pcap_file, const string suffix,
                           unordered_map<int, Matrix4d>& map_rots)
{
    unordered_map<int, int64_t> stamps;
    return load_file(pcap_file, suffix, map_rots, stamps, false);
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map<int, Matrix4d>& map_rots)
{
    return load_file(pcap_file, "", map_rots);
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map<int, Matrix4d>& map_rots,
                               unordered_map<uint64_t, Matrix4d>& map_rots_stamp)
{
    unordered_map<int, int64_t> stamps;
    return load_file_rot(pcap_file, map_rots, map_rots_stamp, stamps);
}

bool FileHelper::load_file_rot(const string pcap_file, unordered_map<int, Matrix4d>& map_rots,
                               unordered_map<uint64_t, Matrix4d>& map_rots_stamp, unordered_map<int, int64_t>& stamps)
{
    bool flag = load_file(pcap_file, "", map_rots, stamps, true);
    for (unordered_map<int, int64_t>::iterator it = stamps.begin(); it != stamps.end(); it++) {
        map_rots_stamp[it->second] = map_rots[it->first];
    }
    return flag;
}

void FileHelper::write_file_rot(const int frame, const string pcap_file,
                                const Matrix4d& trans,
                                bool isAppend, int64_t stamp)
{
    stringstream sstream;
    sstream.precision(std::numeric_limits<double>::digits10);
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    if (stamp != -1) {
        sstream << " " << stamp << " " << 100;
    }
    write_file(pcap_file, sstream.str(), isAppend);
}

void FileHelper::write_file_rot_icp(const int frame, const string pcap_file,
                                    const Matrix4d& trans,
                                    bool isAppend, int64_t stamp, float icp_value)
{
    stringstream sstream;
    sstream.precision(std::numeric_limits<double>::digits10);
    sstream << frame;
    for (int i = 0; i < 4.; i++) {
        for (int j = 0; j < 4; j++) {
            sstream << " " << trans(i, j);
        }
    }
    if (stamp != -1) {
        sstream << " " << stamp << " 100 " << icp_value;
    }
    write_file(pcap_file, sstream.str(), isAppend);
}

void FileHelper::write_file_rot_hdmap(const int frame, const string& file_path,
                                      const Eigen::Matrix4d& trans,
                                      bool isAppend, const double stamp)
{
    stringstream sstream;
    sstream.setf(ios::fixed);
    sstream << frame << " " << stamp << " ";
    Eigen::Affine3d affine(trans);
    Eigen::Quaterniond q(affine.rotation());
    Eigen::Vector3d t(affine.translation());
    t = t;
    sstream << t(0) << " "
        << t(1) << " "
        << t(2) << " "
        << q.x() << " "
        << q.y() << " "
        << q.z() << " "
        << q.w();
    write_file(file_path, sstream.str(), isAppend);
}

void FileHelper::write_file_rot_hdmap(const string& file_path, const vector<CloudStampRot>& rots, bool isAppend)
{
    stringstream sstream;
    sstream.setf(ios::fixed);
    for (int i = 0; i < rots.size(); i++) {
        int frame = rots[i]._stamp - 1;
        double stamp = rots[i]._time_stamp_d;
        const Eigen::Matrix4d &trans = rots[i]._rot;
        sstream << frame << " " << stamp << " ";
        Eigen::Affine3d affine(trans);
        Eigen::Quaterniond q(affine.rotation());
        Eigen::Vector3d t(affine.translation());
        t = t;
        sstream << t(0) << " "
            << t(1) << " "
            << t(2) << " "
            << q.x() << " "
            << q.y() << " "
            << q.z() << " "
            << q.w() << endl;
    }
    write_file(file_path, sstream.str(), isAppend);
}

void FileHelper::write_file_rot_hdmap(const int frame, const string& file_path,
                                      const Eigen::Matrix4d& trans,
                                      bool isAppend, string stamp)
{
    stringstream sstream;
    sstream.setf(ios::fixed);
    sstream << frame << " " << stamp << " ";
    Eigen::Affine3d affine(trans);
    Eigen::Quaterniond q(affine.rotation());
    Eigen::Vector3d t(affine.translation());
    sstream << t(0) << " "
        << t(1) << " "
        << t(2) << " "
        << q.x() << " "
        << q.y() << " "
        << q.z() << " "
        << q.w();
    write_file(file_path, sstream.str(), isAppend);
}

bool FileHelper::get_dir_files(std::string sDirName, std::vector<std::string>& files, string filter)
{
    using namespace boost::filesystem;
    namespace fs = boost::filesystem;
    fs::path fullpath(sDirName);

    if (!fs::exists(fullpath)) {
        return false;
    }
    fs::recursive_directory_iterator end_iter;
    for (fs::recursive_directory_iterator iter(fullpath); iter != end_iter; iter++) {
        try {
            if (fs::is_directory(*iter)) {
                //std::cout<<*iter << "is dir" << std::endl;
                //ret.push_back(iter->path().string());
            } else {
                std::string s_file_name = iter->path().string();
                if (filter != "*") {
                    if (s_file_name.find(filter) == std::string::npos) {
                        continue;
                    }
                }
                //ret.push_back(iter->path().string());
                //std::cout << *iter << " is a file" << std::endl;
                files.push_back(s_file_name);
            }
        }
        catch (const std::exception& ex) {
            std::cerr << ex.what() << std::endl;
            continue;
        }
    }
    return true;
}

bool FileHelper::compare_func(boost::tuple<std::string, uint64_t, uint64_t> p1, boost::tuple<std::string, uint64_t, uint64_t> p2)
{
    if (p1.get<1>() == p2.get<1>()) {
        return p1.get<2>() < p2.get<2>();
    } else {
        return p1.get<1>() < p2.get<1>();
    }
}

bool FileHelper::compare_func2(boost::tuple<std::string, uint64_t> p1, boost::tuple<std::string, uint64_t> p2)
{
    return p1.get<1>() < p2.get<1>();
}

std::vector<std::string> FileHelper::get_all_ply_files(std::string ply_dir)
{
    std::vector<boost::tuple<std::string, uint64_t, uint64_t> > files_pair;
    namespace fs = boost::filesystem3;
    const fs::path path(ply_dir);
    fs::directory_iterator item_begin(path);
    fs::directory_iterator item_end;

    for (; item_begin != item_end; ++item_begin) {
        if (!fs::is_regular_file(*item_begin)) {
            continue;
        }
        fs::path tmp_path = item_begin->path();
        //std::cout << tmp_path.string() << std::endl;
        if (boost::to_lower_copy(tmp_path.extension().string()) != ".ply" && boost::to_lower_copy(tmp_path.extension().string()) != ".pcd") {
            continue;
        }
        std::string filename = tmp_path.filename().string();
        std::vector<std::string> res1;
        boost::split(res1, filename, boost::is_any_of("."));
        if (res1.size() != 2) {
            continue;
        }
        std::vector<std::string> res2;
        boost::split(res2, res1[0], boost::is_any_of("_"));
        if (res2.size() != 2) {
            continue;
        }
        uint64_t index = boost::lexical_cast<uint64_t>(res2[0]);
        uint64_t sub_index = boost::lexical_cast<uint64_t>(res2[1]);
        files_pair.push_back(boost::tuple<std::string, uint64_t, uint64_t>(tmp_path.string(), index, sub_index));
    }
    std::sort(files_pair.begin(), files_pair.end(), compare_func);
    std::vector<std::string> files;
    for (int i = 0; i < files_pair.size(); i++) {
        files.push_back(files_pair[i].get<0>());
    }
    return files;
}

std::vector<std::string> FileHelper::get_all_ply_files2(std::string ply_dir)
{
    std::vector<boost::tuple<std::string, uint64_t> > files_pair;
    namespace fs = boost::filesystem3;
    const fs::path path(ply_dir);
    fs::directory_iterator item_begin(path);
    fs::directory_iterator item_end;

    for (; item_begin != item_end; ++item_begin) {
        if (!fs::is_regular_file(*item_begin)) {
            continue;
        }
        fs::path tmp_path = item_begin->path();
        //std::cout << tmp_path.string() << std::endl;
        if (boost::to_lower_copy(tmp_path.extension().string()) != ".ply" && boost::to_lower_copy(tmp_path.extension().string()) != ".pcd") {
            continue;
        }
        std::string filename = tmp_path.filename().string();
        std::vector<std::string> res1;
        boost::split(res1, filename, boost::is_any_of("."));
        if (res1.size() != 2) {
            continue;
        }
        std::cout << "get pcd path " << filename << " " << res1[0] << " " << res1[1] << std::endl;
        uint64_t index = boost::lexical_cast<uint64_t>(res1[0]);
        files_pair.push_back(boost::tuple<std::string, uint64_t>(tmp_path.string(), index));
    }
    std::sort(files_pair.begin(), files_pair.end(), compare_func2);
    std::vector<std::string> files;
    for (int i = 0; i < files_pair.size(); i++) {
        files.push_back(files_pair[i].get<0>());
    }
    return files;
}

bool FileHelper::get_index_stamp_from_filename(const string& file_path, uint64_t& index, uint64_t& sub_index, uint64_t& stamp)
{
    boost::filesystem3::path temp_path(file_path);
    string filename = temp_path.filename().string();
    std::vector<std::string> res1;
    boost::split(res1, filename, boost::is_any_of("."));
    if (res1.size() != 2) {
        return false;
    }
    std::vector<std::string> res2;
    boost::split(res2, res1[0], boost::is_any_of("_"));
    if (res2.size() != 2) {
        return false;
    }
    index = boost::lexical_cast<uint64_t>(res2[0]);
    sub_index = boost::lexical_cast<uint64_t>(res2[1]);

    stamp = index * 1000000LL + sub_index;
    return true;
}

bool FileHelper::get_index_stamp_from_filename2(const string& file_path, uint64_t& index, uint64_t& stamp)
{
    boost::filesystem3::path temp_path(file_path);
    string filename = temp_path.filename().string();
    std::vector<std::string> res1;
    boost::split(res1, filename, boost::is_any_of("."));
    if (res1.size() != 2) {
        return false;
    }
    index = boost::lexical_cast<uint64_t>(res1[0]);

    stamp = index;
    return true;
}

bool FileHelper::load_file_rot_icp_hdmap(const string file_path, vector<CloudStampRot>& stamp_rots, string& proj_str)
{
    fstream in_file(file_path.c_str());
    if (!in_file.good()) {
        cout << "file rot empty :" << file_path << endl;
        return false;
    }
    long pointcount = 0;
    double tempTime = 0.0;
    string str_line;
    vector<string> arr_strs;
    uint64_t stamp = 0;
    int frame = 0;
    double value_icp = -2;
    std::getline(in_file, proj_str);
    while (!in_file.eof()) {
        getline(in_file, str_line, '\n');
        if (in_file.fail()) {
            in_file.clear(in_file.rdstate() & ~(ifstream::failbit));
        }
        arr_strs = StringHelper::split(str_line, " ");
        if (arr_strs.size() == 9) {
            frame = atoi(arr_strs[0].c_str());
            pointcount++;

            //Eigen::Quaterniond qua(S2D(arr_strs[8]),S2D(arr_strs[5]),S2D(arr_strs[6]),
            //	S2D(arr_strs[7]));
            //Eigen::Vector3d vec3_d( -483189.100569 ,-3371741.645173 ,-430.094726 );
            //Eigen::Vector3d vec3(S2D(arr_strs[2]) ,S2D(arr_strs[3]),S2D(arr_strs[4]));
            //Eigen::Translation3d t3(vec3 + vec3_d);
            //Eigen::Affine3d affine_l_2_w(t3 * qua);
            Eigen::Affine3d affine_l_2_w2 = Eigen::Affine3d(Eigen::Quaterniond(S2D(arr_strs[8]),S2D(arr_strs[5]),
                                                                                               S2D(arr_strs[6]),S2D(arr_strs[7]))).pretranslate(Eigen::Vector3d(
                S2D(arr_strs[2]), S2D(arr_strs[3]), S2D(arr_strs[4])));
            Matrix4d trans = Eigen::Matrix4d::Identity();
            //trans=affine_l_2_w * trans;
            //cout.setf(ios::fixed);
            //cout << affine_l_2_w.matrix() << endl << affine_l_2_w2.matrix() << endl;
            //if (pointcount<10)
            //{
            //	cout<<trans<<endl;
            //}
            //trans.block<1,3>(0,3)  = trans.block<1,3>(0,3);
            //cout<<trans<<endl;
            //Eigen::Matrix3d rot = affine_l_2_w2.rotation();
            //Eigen::Vector3d t = affine_l_2_w2.translation();
            //Eigen::Quaterniond q(rot);
            //Eigen::Vector3d tr = rot.inverse() * t - vec3_d;
            trans = affine_l_2_w2.matrix();
            //cout << vec3 << endl;
            //cout << t - vec3_d << endl << endl;
            double stamp_real = S2D(arr_strs[1]);
            string stamp_real_str = arr_strs[1];
            stringstream stream;
            stream << arr_strs[0];
            stream >> stamp;
            /*
            auto affine_l_2_w = Eigen::Affine3d(Eigen::Quaterniond(pos[index].posture().w(),
            pos[index].posture().x(), pos[index].posture().y(),
            pos[index].posture().z())).pretranslate(Eigen::Vector3d(
            pos[index].position().x(), pos[index].position().y(), pos[index].position().z()));

            Eigen::Vector3d p = affine_l_2_w * Eigen::Vector3d(point.x, point.y, point.z);*/

            CloudPtr cloud(new Cloud);
            cloud->header.stamp = stamp + 1;
            //Eigen::Matrix4d rotf = trans.cast<float>();
            CloudStampRot stamp_rot(cloud->header.stamp, trans, cloud, value_icp);
            stamp_rot._time_stamp_d = stamp_real;
            //stamp_rot._rotd = trans;
            stamp_rot._time_stamp_str = stamp_real_str;
            //cout<<pointcount<<endl<<stamp_rot._rotd<<endl;
            stamp_rots.push_back(stamp_rot);
        } else {
            continue;
        }
    }
    in_file.close();

    return true;
}

CLOUD_BLEND_DOUBLE_NAMESPACE_END