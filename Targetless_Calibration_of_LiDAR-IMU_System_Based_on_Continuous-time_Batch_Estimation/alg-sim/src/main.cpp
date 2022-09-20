#include "calib.h"
#include "odometer.h"
#include "sensor.h"
#include "imu_bspline.h"

int main(int argc, char **argv) {
    try {
        auto calibSolver = ns_calib::CalibSolver::create();
        calibSolver->solve();

    } catch (const ns_calib::Status &status) {
        std::cout << status << std::endl;
    }
    return 0;
}