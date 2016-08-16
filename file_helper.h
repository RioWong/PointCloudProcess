#pragma once
#include <boost/unordered_map.hpp>
#include <boost/tuple/tuple.hpp>
#include <map>
#include "cloud_stamp_rot.h"

CLOUD_BLEND_DOUBLE_NAMESPACE_BEGIN
class FileHelper
{
public:
    FileHelper();
    ~FileHelper();

    static bool load_file(const std::string pcap_file, const std::string suffix,
                          boost::unordered_map<int, Eigen::Matrix4d>& map_rots);
    static bool load_file_rot(const std::string pcap_file,
                              boost::unordered_map<int, Eigen::Matrix4d>& map_rots,
                              boost::unordered_map<uint64_t, Eigen::Matrix4d>& map_rots_stamp);
    static bool load_file_rot(const std::string pcap_file,
                                          boost::unordered_map<int, Eigen::Matrix4d>& map_rots,
                                          boost::unordered_map<uint64_t, Eigen::Matrix4d>& map_rots_stamp,
                                          boost::unordered_map<int, int64_t>& stamps);
    static void write_file(const std::string filePath, const std::string content, bool isAppend);
    static void write_file_rot(const int frame, const std::string pcap_file,
                               const Eigen::Matrix4d& trans, bool isAppend, int64_t stamp = -1);
    static bool load_file(const std::string pcap_file, const std::string suffix,
                          boost::unordered_map<int, Eigen::Matrix4d>& map_rots,
                          boost::unordered_map<int, int64_t>& stamps, bool isContainStamp);
    static bool load_file_rot(const std::string pcap_file,
                              boost::unordered_map<int, Eigen::Matrix4d>& map_rots);
    static bool load_file_ignore_frame(const std::string pcap_file,
                                       std::map<uint64_t, uint64_t>& stamps);
    static bool get_dir_files(std::string sDirName, std::vector<std::string>& files, std::string filter);
    static std::vector<std::string> get_all_ply_files(std::string ply_dir);
    static std::vector<std::string> get_all_ply_files2(std::string ply_dir);
    static bool get_index_stamp_from_filename(const std::string& file_path, uint64_t& index, uint64_t& sub_index, uint64_t& stamp);
    static bool get_index_stamp_from_filename2(const std::string& file_path, uint64_t& index, uint64_t& stamp);
    static bool load_file_rot_icp(const std::string pcap_file, std::vector<CloudStampRot>& stamp_rots);
    static void write_file_rot_icp(const int frame, const std::string pcap_file, const Eigen::Matrix4d& trans, bool isAppend, int64_t stamp, float icp_value);
    static void write_file_rot_hdmap(const int frame,
                                     const std::string& file_path, const Eigen::Matrix4d& trans, bool isAppend, const double stamp);
    static void write_file_rot_hdmap(const int frame,
                                     const std::string& file_path, const Eigen::Matrix4d& trans, bool isAppend, std::string stamp);
    static void write_file_rot_hdmap(const std::string& file_path, const std::vector<CloudStampRot>& rots, bool isAppend);
    static bool load_file_rot_icp_hdmap(const std::string file_path, std::vector<CloudStampRot>& stamp_rots, std::string& proj_str);
    static bool copy_to_file_subfix(const std::string& file_path, const std::string& subfix);
private:
    static bool compare_func(boost::tuple<std::string, uint64_t, uint64_t> p1, boost::tuple<std::string, uint64_t, uint64_t> p2);
    static bool compare_func2(boost::tuple<std::string, uint64_t> p1, boost::tuple<std::string, uint64_t> p2);
    DISALLOW_COPY_AND_ASSIGN(FileHelper);
};

} // cloud_icp_reg