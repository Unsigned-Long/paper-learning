//
// Created by csl on 9/21/22.
//

#include "config.h"

namespace ns_calib {
    Eigen::Vector3f Config::LiDAROdometer::AVGFilter::LEAF_SIZE = Eigen::Vector3f(0.2f, 0.2f, 0.3f);

    double Config::LiDAROdometer::NDT::TRANS_EPSILON = 0.01;
    double Config::LiDAROdometer::NDT::STEP_SIZE = 0.1;
    float Config::LiDAROdometer::NDT::RESOLUTION = 1.0f;
    int Config::LiDAROdometer::NDT::MAX_ITERATIONS = 35;

    bool Config::initConfig() {
        return true;
    }
}