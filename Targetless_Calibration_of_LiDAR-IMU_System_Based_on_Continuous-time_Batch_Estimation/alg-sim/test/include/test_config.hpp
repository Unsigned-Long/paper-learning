//
// Created by csl on 9/21/22.
//

#ifndef ALG_SIM_TEST_CONFIG_HPP
#define ALG_SIM_TEST_CONFIG_HPP

#include "gtest/gtest.h"
#include "config.h"

TEST(config, Config) {
    using namespace ns_calib;
    EXPECT_DOUBLE_EQ(Config::LiDAROdometer::NDT::TRANS_EPSILON, 0.001);
    EXPECT_DOUBLE_EQ(Config::LiDAROdometer::NDT::STEP_SIZE, 0.1);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::NDT::RESOLUTION, 2.0f);
    EXPECT_EQ(Config::LiDAROdometer::NDT::MAX_ITERATIONS, 50);

    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[0], 0.5f);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[1], 0.5f);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[2], 0.5f);

    EXPECT_FLOAT_EQ(Config::LiDAROdometer::UpdateMapThd, 2.0f);
}

#endif //ALG_SIM_TEST_CONFIG_HPP
