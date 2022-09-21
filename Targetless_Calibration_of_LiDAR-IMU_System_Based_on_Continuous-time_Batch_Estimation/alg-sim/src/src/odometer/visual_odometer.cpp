//
// Created by csl on 9/21/22.
//

#include "odometer/visual_odometer.h"

namespace ns_calib {

    /*
     * VisualOdometer
     */
    VisualOdometer::VisualOdometer(const MonoPinholeCamera &sensor) : Odometer(sensor) {}

    VisualOdometer::Ptr VisualOdometer::create(const MonoPinholeCamera &sensor) {
        return std::make_shared<VisualOdometer>(sensor);
    }

    bool VisualOdometer::feedObservation(Sensor::ObvPtr obv) {
        return false;
    }
}