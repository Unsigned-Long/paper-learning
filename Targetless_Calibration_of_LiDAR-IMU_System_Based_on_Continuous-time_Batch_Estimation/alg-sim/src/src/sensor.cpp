//
// Created by csl on 9/20/22.
//

#include "sensor.h"

namespace ns_calib {

    SensorType LiDAR::type() const {
        return SensorType::LiDAR_3D;
    }


    SensorType IMU::type() const {
        return SensorType::IMU_MEMS;
    }


    SensorType MonoPinholeCamera::type() const {
        return SensorType::CAMERA_PINHOLE;
    }

}