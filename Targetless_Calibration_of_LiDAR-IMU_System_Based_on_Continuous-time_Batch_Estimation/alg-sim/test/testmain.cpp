//
// Created by csl on 9/21/22.
//

#include "test_config.hpp"

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    EXPECT_EQ(ns_calib::Config::initConfig("../config/config.yaml"), true);
    return RUN_ALL_TESTS();
}