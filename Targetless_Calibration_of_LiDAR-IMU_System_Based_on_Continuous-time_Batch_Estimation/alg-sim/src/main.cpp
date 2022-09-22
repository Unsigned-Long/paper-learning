#include "calib.h"
#include "odometer/lidar_odometer.h"
#include "odometer/visual_odometer.h"
#include "imu_bspline.h"
#include "fstream"
#include "thread"
#include "artwork/logger/logger.h"


int main(int argc, char **argv) {
    try {
        // load configure file
        if (!ns_calib::Config::initConfig("../config/config.yaml")) {
            throw ns_calib::Status(
                    ns_calib::Status::Flag::ERROR,
                    "Initialize Configure Failed in 'Config::initConfig'."
            );
        }

    } catch (const ns_calib::Status &status) {
        std::cout << status << std::endl;
    }
    return 0;
}