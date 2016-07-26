#include "stdafx.h"

#include <boost/program_options.hpp>

#include "cloud_grid.h"
#include "file_helper.h"
#include "string_helper.h"
#include "point_cloud_helper.h"
#include "point_cloud_spliter.h"
#include "point_cloud_closure.h"
#include "point_cloud_revert.h"
#include "params.h"
#include "main.h"

bool check_params_unique_valid(
        const boost::program_options::variables_map& vm, 
        const boost::program_options::options_description& desc)
{
    for (int i = 0; i < desc.options().size(); i++) {
        const std::string name = desc.options()[i]->long_name();
        if (vm.count(name) != 1) {
            return false;
        }
    }
    return true;
}

bool check_params_unique_valid(
        const boost::program_options::variables_map& vm, 
        const std::vector<std::string>& desc)
{
    for (int i = 0; i < desc.size(); i++) {
        if (vm.count(desc[i]) != 1) {
            return false;
        }
    }
}

void display_help_message(
        const boost::program_options::options_description &general_desc,
        const boost::program_options::options_description &split_desc,
        const boost::program_options::options_description &blend_desc,
        const boost::program_options::options_description &closure_desc)
{
    std::cout << general_desc << std::endl;

    std::cout << std::endl << "closure mode" << std::endl;
    std::cout << closure_desc << std::endl;

    std::cout << std::endl << "split mode" << std::endl;
    std::cout << split_desc << std::endl;

    std::cout << std::endl << "blend mode" << std::endl;
    std::cout << blend_desc << std::endl;
}

int run_split_command(
        int argc, 
        char** argv, 
        const boost::program_options::options_description &split_desc)
{
    namespace po = boost::program_options;
    po::variables_map split_vm;
    try {
        // 解析split命令
        po::store(
            po::command_line_parser(argc, argv).options(split_desc).allow_unregistered().run(), 
            split_vm);
        po::notify(split_vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        std::cout << split_desc << std::endl;
        return 0;
    } catch (...) {
        std::cout << split_desc << std::endl;
        return 0;
    }
    // 如果是split
    if (check_params_unique_valid(split_vm, split_desc)) {
        std::string pcd_dir = split_vm["in_dir_src"].as<std::string>();
        std::string base_pose_file = split_vm["in_file_base_pose"].as<std::string>();
        std::string frame_pose_file = split_vm["in_file_frame_pose"].as<std::string>();
        std::string out_dir = split_vm["out_dir"].as<std::string>();
        std::string out_pose_file = split_vm["out_file_pose"].as<std::string>();
        std::string out_stamp_file = split_vm["out_file_stamp"].as<std::string>();
        cloud_blend_double::split_params params(
                pcd_dir,
                base_pose_file,
                frame_pose_file,
                out_dir,
                out_pose_file,
                out_stamp_file);
        return CLOUD_BLEND_DOUBLE_NAMESPACE::PointCloudSpliter::main(params);
    } else {
        std::cout << split_desc << std::endl;
    }
    return 0;
}

int run_blend_command(int argc, char** argv, const boost::program_options::options_description &blend_desc)
{
    namespace po = boost::program_options;
    po::variables_map blend_vm;
    try {
        po::store(
            po::command_line_parser(argc, argv).options(blend_desc).allow_unregistered().run(), 
            blend_vm);
        po::notify(blend_vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        std::cout << blend_desc << std::endl;
        return 0;
    } catch (...) {
        std::cout << blend_desc << std::endl;
        return 0;
    }
    // 如果是blend
    if (check_params_unique_valid(blend_vm, blend_desc)) {
        std::string base_dir = blend_vm["in_dir_base"].as<std::string>();
        std::string frame_dir = blend_vm["in_dir_frame"].as<std::string>();
        std::string stamp_file = blend_vm["in_file_stamp"].as<std::string>();
        std::string in_pose_file = blend_vm["in_file_pose"].as<std::string>();
        std::string out_pose_file = blend_vm["out_file_pose"].as<std::string>();
        std::string cloud_dir = blend_vm["out_dir_cloud"].as<std::string>();
        bool is_same_direction = blend_vm["is_same_direction"].as<bool>();
        double icp_threshold = is_same_direction ? 0.13 : 0.22;
        int cache_frame_num = is_same_direction ? 10 : 17;
        CLOUD_BLEND_DOUBLE_NAMESPACE::blend_params params(
                base_dir,
                frame_dir,
                stamp_file,
                in_pose_file,
                cloud_dir,
                out_pose_file,
                is_same_direction,
                icp_threshold,
                cache_frame_num);
        int ret = CLOUD_BLEND_DOUBLE_NAMESPACE::main_blend(CLOUD_BLEND_DOUBLE_NAMESPACE::ENUM_MODE_PROCESS_POS, params);
        if (ret != 0) {
            return ret;
        }
        ret = CLOUD_BLEND_DOUBLE_NAMESPACE::main_blend(CLOUD_BLEND_DOUBLE_NAMESPACE::ENUM_MODE_PROCESS_LOOP, params);
        if (ret != 0) {
            return ret;
        }
    } else {
        std::cout << blend_desc << std::endl;
    }
    return 0;
}

int run_convert_command(int argc, char** argv,
                        const boost::program_options::options_description& convert_desc) {
    namespace po = boost::program_options;
    po::variables_map convert_vm;

    try {
        po::store(
            po::command_line_parser(argc, argv).options(convert_desc).allow_unregistered().run(),
            convert_vm);
        po::notify(convert_vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        std::cout << convert_desc << std::endl;
        return 0;
    } catch (...) {
        std::cout << convert_desc << std::endl;
        return 0;
    }

    if (check_params_unique_valid(convert_vm, convert_desc)) {
        std::string in_pose_log = convert_vm["in_pose_log"].as<std::string>();
        std::string in_start_pcd = convert_vm["in_start_pcd"].as<std::string>();
        std::string in_end_pcd = convert_vm["in_end_pcd"].as<std::string>();
        std::string out_pcd = convert_vm["out_pcd"].as<std::string>();
        std::string center_point = convert_vm["center_point"].as<std::string>();
        cloud_blend_double::convert_params params(in_pose_log, in_start_pcd, in_end_pcd, out_pcd,
                center_point);
        return cloud_blend_double::PointCloudRevert::main(params);
    } else {
        std::cout << convert_desc << std::endl;
    }

    return 0;
}

int run_closure_command(int argc, char** argv,
                        const boost::program_options::options_description& closure_desc) {
    namespace po = boost::program_options;
    po::variables_map closure_vm;

    try {
        po::store(
            po::command_line_parser(argc, argv).options(closure_desc).allow_unregistered().run(),
            closure_vm);
        po::notify(closure_vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        std::cout << closure_desc << std::endl;
        return 0;
    } catch (...) {
        std::cout << closure_desc << std::endl;
        return 0;
    }

    if (check_params_unique_valid(closure_vm, closure_desc)) {
        std::string in_file_pose = closure_vm["in_file_pose"].as<std::string>();
        std::string in_dir_pcd = closure_vm["in_dir_pcd"].as<std::string>();
        std::string out_file_pose = closure_vm["out_file_pose"].as<std::string>();
        std::string out_dir_base = closure_vm["out_dir_base"].as<std::string>();
        std::string out_dir_frame = closure_vm["out_dir_frame"].as<std::string>();
        std::string out_dir_cloud = closure_vm["out_dir_cloud"].as<std::string>();
        cloud_blend_double::closure_params params(in_file_pose, in_dir_pcd, out_dir_base, out_dir_frame,
                out_dir_cloud, out_file_pose);
        return cloud_blend_double::PointCloudClosure::main(params);
    } else {
        std::cout << closure_desc << std::endl;
    }

    return 0;
}

int main(int argc, char** argv){
    //cloud_blend_double::PointCloudRevert::main(argc, argv);
    //return 0;
    time_t tm_tmp = time(NULL);
    tm* p_time = localtime(&tm_tmp);
    if (p_time->tm_year + 1900 > 2016) {
        return 0;
    }

    namespace po = boost::program_options;

    // 统一命令行参数
    po::options_description general_desc("hdmap point cloud split and blend program");
    general_desc.add_options()
        ("help,h", "show this help message")
        ("mode,m", po::value<std::string>(), "\"closure\" for point cloud loop closure mode, \"split\" for point cloud split mode, \"blend\" for point cloud blend mode.");

    po::options_description closure_desc("parameters for point cloud loop closure mode[--mode closure]");
    closure_desc.add_options()
        ("in_file_pose,p", po::value<std::string>(), "[in]origin pose file path")
        ("in_dir_pcd,P", po::value<std::string>(), "[in]source pcd dir path")
        ("out_file_pose,o", po::value<std::string>(), "[out]optimized pose file path")
        ("out_dir_base,b", po::value<std::string>(), "[out]base point cloud dir, temporarily")
        ("out_dir_frame,f", po::value<std::string>(), "[out]frame point cloud dir, temporarily")
        ("out_dir_cloud,c", po::value<std::string>(), "[out]output point cloud dir, temporarily");

    // split命令行参数
    po::options_description split_desc("parameters for point cloud split mode[--mode split]");
    split_desc.add_options()
        ("in_dir_src,s", po::value<std::string>(), "[in]source pcd dir path")
        ("in_file_base_pose,b", po::value<std::string>(), "[in]base pose file path")
        ("in_file_frame_pose,f", po::value<std::string>(), "[in]frame pose file path")
        ("out_dir,d", po::value<std::string>(), "[out]output splited-pcd directory path")
        ("out_file_pose,p", po::value<std::string>(), "[out]output pose file path")
        ("out_file_stamp,S", po::value<std::string>(), "[out]output sliced stamp file path");

    // blend命令行参数
    po::options_description blend_desc("parameters for point cloud blend mode[--mode blend]");
    blend_desc.add_options()
        ("in_dir_base,b", po::value<std::string>(), "[in]base splited-pcd dir")
        ("in_dir_frame,f", po::value<std::string>(), "[in]frame splited-pcd dir")
        ("in_file_stamp,s", po::value<std::string>(), "[in]sliced stamp file of the splited point cloud")
        ("is_same_direction,d", po::value<bool>(), "[in]whether the base point cloud and the frame point cloud the same direction")
        ("in_file_pose,p", po::value<std::string>(), "[in]splited pose file")
        ("out_file_pose,P", po::value<std::string>(), "[out]output pose file prefix")
        ("out_dir_cloud,c", po::value<std::string>(), "[out]output point cloud data directory of the result");

    po::options_description convert_desc("parameters for point cloud convert mode[--mode convert]");
    convert_desc.add_options()
        ("in_pose_log,p", po::value<std::string>(), "[in]input pose log file")
        ("in_start_pcd,s", po::value<std::string>(), "[in]start pcd file")
        ("in_end_pcd,e", po::value<std::string>(), "[in]end pcd file")
        ("center_point,c", po::value<std::string>(), "[in]center point, \"x,y,z\"")
        ("out_pcd,o", po::value<std::string>(), "[out]output pcd file");


    po::variables_map general_vm;
    try {
        po::store(
            po::command_line_parser(argc, argv).options(general_desc).allow_unregistered().run(), 
            general_vm);
        po::notify(general_vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        display_help_message(general_desc, split_desc, blend_desc, closure_desc);
        return 0;
    } catch (...) {
        display_help_message(general_desc, split_desc, blend_desc, closure_desc);
        return 0;
    }

    if (general_vm.count("help") || general_vm.count("mode") != 1) {
        // 参数中含有help参数
        // 参数中mode的个数不为1
        display_help_message(general_desc, split_desc, blend_desc, closure_desc);
        return 0;
    } else {
        std::string mode = boost::to_lower_copy(general_vm["mode"].as<std::string>());
        int ret = 0;
        if (mode == std::string("split")) {
            ret = run_split_command(argc, argv, split_desc);
        } else if (mode == std::string("blend")) {
            ret = run_blend_command(argc, argv, blend_desc);
        } else if (mode == std::string("convert")) {
            ret = run_convert_command(argc, argv, convert_desc);
        } else if (mode == std::string("closure")) {
            ret = run_closure_command(argc, argv, closure_desc);
        } else {
            display_help_message(general_desc, split_desc, blend_desc, closure_desc);
            ret = 0;
        }
        return ret;
    }

    return 0;
}
