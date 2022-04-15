#include "yaml-cpp/yaml.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>


int main()
{
    YAML::Emitter out;
    out << "Hello, World!";
    std::cout << "Here's the output YAML:" << out.c_str() << std::endl; // prints "Hello, World!"

    YAML::Node config = YAML::LoadFile
            ("/home/hj/VINS_from_scratch/io/config/realsense_infra1_config.yaml");

    if (config["imu_topic"]) {
    std::cout << "imu topic: " << config["imu_topic"].as<std::string>() << "\n";
    }
    std::string imu_topic = config["imu_topic"].as<std::string>();
    std::string image_topic = config["image_topic"].as<std::string>();
    std::string output_path = config["output_path"].as<std::string>();

    const std::string model_type = config["model_type"].as<std::string>();
    const std::string camera_name = config["camera_name"].as<std::string>();
    const int image_width = config["image_width"].as<int>();
    const int image_height = config["image_height"].as<int>();

    std::vector<double> distortion_parameters; 
    distortion_parameters.emplace_back(config["distortion_parameters"]["k1"].as<double>());
    distortion_parameters.emplace_back(config["distortion_parameters"]["k2"].as<double>());
    distortion_parameters.emplace_back(config["distortion_parameters"]["p1"].as<double>());
    distortion_parameters.emplace_back(config["distortion_parameters"]["p2"].as<double>());

    std::vector<double> projection_parameters; 
    projection_parameters.emplace_back(config["projection_parameters"]["fx"].as<double>());
    projection_parameters.emplace_back(config["projection_parameters"]["fy"].as<double>());
    projection_parameters.emplace_back(config["projection_parameters"]["cx"].as<double>());
    projection_parameters.emplace_back(config["projection_parameters"]["cy"].as<double>());

    const int estimate_extrinsic = config["estimate_extrinsic"].as<int>();

    Eigen::Matrix3d extrinsicRotation(config["extrinsicRotation"]["data"]
                                                .as<std::vector<double>>().data());
    Eigen::Vector3d extrinsicTranslation(config["extrinsicTranslation"]["data"]
                                                .as<std::vector<double>>().data());
    std::cout << extrinsicRotation << std::endl;
    std::cout << extrinsicTranslation << std::endl;

    const int max_cnt =     config["max_cnt"].as<int>();
    const int min_dist =    config["min_dist"].as<int>();
    const int freq =        config["freq"].as<int>();
    const double F_threshold = config["F_threshold"].as<double>();
    const int show_track = config["show_track"].as<int>();
    const int equalize =   config["equalize"].as<int>();
    const int fisheye =    config["fisheye"].as<int>();

    // optimization parameters
    const double max_solver_time = config["max_solver_time"].as<double>();
    const double max_num_iterations = config["max_num_iterations"].as<double>();
    const double keyframe_parallax = config["keyframe_parallax"].as<double>();

    // imu parameters. The more accurate parameters you provide, the better performance    
    const double acc_n = config["acc_n"].as<double>();
    const double gyr_n = config["gyr_n"].as<double>();
    const double acc_w = config["acc_w"].as<double>();
    const double gyr_w = config["gyr_w"].as<double>();
    const double g_norm = config["g_norm"].as<double>();

    // loop closure parameters
    const int loop_closure = config["loop_closure"].as<int>();
    const int fast_relocalization = config["fast_relocalization"].as<int>();
    const int load_previous_pose_graph = config["load_previous_pose_graph"].as<int>();
    const std::string pose_graph_save_path = config["pose_graph_save_path"].as<std::string>();

    // unsynchronization parameters
    const int estimate_td = config["estimate_td"].as<int>();
    const double td = config["td"].as<double>();

    // rolling shutter parameters
    const int rolling_shutter = config["rolling_shutter"].as<int>();
    const int rolling_shutter_tr = config["rolling_shutter_tr"].as<int>();

    // visualization parameters
    const int save_image = config["save_image"].as<int>();
    const int visualize_imu_forward = config["visualize_imu_forward"].as<int>();
    const double visualize_camera_size = config["visualize_camera_size"].as<double>();


   return 0;
}