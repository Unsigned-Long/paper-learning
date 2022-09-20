//
// Created by csl on 9/20/22.
//

#include "odometer.h"

namespace ns_calib {

    LiDAROdometer::LiDAROdometer(const LiDAR &lidar) : Odometer(lidar) {}

    bool LiDAROdometer::feedObservation(LiDAR::ObvPtr obv) {
        return false;
    }

    VisualOdometer::VisualOdometer(const MonoPinholeCamera &monoPinholeCamera) : Odometer(monoPinholeCamera) {}

    bool VisualOdometer::feedObservation(MonoPinholeCamera::ObvPtr obv) {
        return false;
    }
}