//
// Created by csl on 9/24/22.
//

#ifndef SFM_CONFIG_HPP
#define SFM_CONFIG_HPP

#include "string"

struct Config {
    static std::string outputDir;
    static std::string imgDir;
    static std::string fileDatabasePath;
    // choice: INCREMENTAL INCREMENTALV2 GLOBAL
    static std::string engineName;
};

std::string Config::imgDir = "../img";
std::string Config::outputDir = "../new_output";
std::string Config::fileDatabasePath = "../config/sensor_width_camera_database.txt";
std::string Config::engineName = "GLOBAL";

#endif //SFM_CONFIG_HPP
