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
                // minimum transformation difference for termination condition
                static double TRANS_EPSILON;
                // maximum step size for More-Thuente line search
                static double STEP_SIZE;
                // Resolution of NDT grid structure (VoxelGridCovariance)
                static float RESOLUTION;
                // max number of registration iterations
                static int MAX_ITERATIONS;

            };
            struct AVGFilter {
                // Approximate Voxel Grid leaf size
                static Eigen::Vector3f LEAF_SIZE;
            };
            // update map only when the distance is bigger than the 'UpdateMapThd', unit: Meter
            static float UpdateMapThd;
        };

    public:
        static bool initConfig(const std::string &configFilePath);
    };
}


#endif //ALG_SIM_CONFIG_H
