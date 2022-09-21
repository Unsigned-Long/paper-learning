#include "calib.h"
#include "odometer/lidar_odometer.h"
#include "odometer/visual_odometer.h"
#include "imu_bspline.h"

int main(int argc, char **argv) {
    try {
        if (!ns_calib::Config::initConfig()) {
            throw ns_calib::Status(
                    ns_calib::Status::Flag::ERROR,
                    "Initialize Configure Failed in 'Config::initConfig'."
            );
        }
        auto calibSolver = ns_calib::CalibSolver::create();
        calibSolver->solve();

    } catch (const ns_calib::Status &status) {
        std::cout << status << std::endl;
    }
    return 0;
}