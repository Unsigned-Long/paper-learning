//
// Created by csl on 9/21/22.
//

#ifndef ALG_SIM_CONFIG_H
#define ALG_SIM_CONFIG_H

#include "Eigen/Core"

namespace ns_calib {
    struct Config {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct LiDAROdometer {
            struct NDT {
                static double TRANS_EPSILON;
                static double STEP_SIZE;
                static float RESOLUTION;
                static int MAX_ITERATIONS;

            };
            struct AVGFilter {
                static Eigen::Vector3f LEAF_SIZE;
            };
        };

    public:
        static bool initConfig();
    };
}


#endif //ALG_SIM_CONFIG_H
