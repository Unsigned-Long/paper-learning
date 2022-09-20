//
// Created by csl on 9/20/22.
//

#include "imu_bspline.h"

namespace ns_calib {

    IMUBSpline::IMUBSpline(const IMU &imu) : Odometer(imu) {}

    bool IMUBSpline::feedObservation(IMU::ObvPtr obv) {
        this->raw_data.push_back(obv);
        return true;
    }

    bool IMUBSpline::computeBSpline() {
        return false;
    }

    double IMUBSpline::startTimeStamp() const {
        return this->raw_data.front()->timeStamp;
    }

    double IMUBSpline::endTimeStamp() const {
        return this->raw_data.back()->timeStamp;
    }
}