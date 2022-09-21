//
// Created by csl on 9/21/22.
//

#ifndef ALG_SIM_TEST_CONFIG_HPP
#define ALG_SIM_TEST_CONFIG_HPP

#include "gtest/gtest.h"
#include "config.h"

TEST(config, Config) {
    using namespace ns_calib;
    EXPECT_DOUBLE_EQ(Config::LiDAROdometer::NDT::TRANS_EPSILON, 0.01);
    EXPECT_DOUBLE_EQ(Config::LiDAROdometer::NDT::STEP_SIZE, 0.1);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::NDT::RESOLUTION, 1.0f);
    EXPECT_EQ(Config::LiDAROdometer::NDT::MAX_ITERATIONS, 35);

    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[0], 0.2f);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[1], 0.2f);
    EXPECT_FLOAT_EQ(Config::LiDAROdometer::AVGFilter::LEAF_SIZE[2], 0.2f);
}

#endif //ALG_SIM_TEST_CONFIG_HPP
