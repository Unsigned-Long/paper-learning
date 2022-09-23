//
// Created by csl on 9/21/22.
//

#include "config.h"
#include "yaml-cpp/yaml.h"

namespace ns_calib {
    /*
     * fields
     */
    Eigen::Vector3f Config::LiDAROdometer::AVGFilter::LEAF_SIZE = {};

    double Config::LiDAROdometer::NDT::TRANS_EPSILON = {};
    double Config::LiDAROdometer::NDT::STEP_SIZE = {};
    float Config::LiDAROdometer::NDT::RESOLUTION = {};
    int Config::LiDAROdometer::NDT::MAX_ITERATIONS = {};

    float Config::LiDAROdometer::UpdateMapThd = {};

    bool Config::initConfig(const std::string &configFilePath) {
        auto config = YAML::LoadFile(configFilePath);
        if (!config["LiDAROdometer"]) {
            return false;
        }
        auto lidarOdometer = config["LiDAROdometer"];

        auto NDT = lidarOdometer["NDT"];
        LiDAROdometer::NDT::TRANS_EPSILON = NDT["TRANS_EPSILON"].as<double>();
        LiDAROdometer::NDT::STEP_SIZE = NDT["STEP_SIZE"].as<double>();
        LiDAROdometer::NDT::RESOLUTION = NDT["RESOLUTION"].as<float>();
        LiDAROdometer::NDT::MAX_ITERATIONS = NDT["MAX_ITERATIONS"].as<int>();

        auto AVGFilter = lidarOdometer["AVGFilter"];
        auto leafSize = AVGFilter["LEAF_SIZE"];
        LiDAROdometer::AVGFilter::LEAF_SIZE(0) = leafSize[0].as<float>();
        LiDAROdometer::AVGFilter::LEAF_SIZE(1) = leafSize[1].as<float>();
        LiDAROdometer::AVGFilter::LEAF_SIZE(2) = leafSize[2].as<float>();

        Config::LiDAROdometer::UpdateMapThd = lidarOdometer["UpdateMapThd"].as<float>();

        return true;
    }
}