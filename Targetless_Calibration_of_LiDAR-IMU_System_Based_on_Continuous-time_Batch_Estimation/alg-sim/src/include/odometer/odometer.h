//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_ODOMETER_H
#define ALG_SIM_ODOMETER_H

#include "sensor/sensor.h"
#include "config.h"
#include "sophus/so3.hpp"

namespace ns_calib {
    template<class SensorType>
    class Odometer {
    public:
        using Sensor = SensorType;
        using SensorPtr = typename Sensor::Ptr;

        using Ptr = std::shared_ptr<Odometer<Sensor>>;

    private:
        Sensor sensor;

    public:
        explicit Odometer(const Sensor &sensor) : sensor(sensor) {}

    };

    template<class Scalar>
    struct OdometerPose {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Rotation = Sophus::SO3<Scalar>;
        using Translation = Eigen::Vector3<Scalar>;

        Rotation so3;
        Translation t;

        OdometerPose(const Rotation &r, const Translation &t) : so3(r), t(t) {}

        OdometerPose() = default;

        Eigen::Quaternion<Scalar> q() const {
            return so3.unit_quaternion();
        }

        Eigen::Matrix3<Scalar> R() const {
            return q().toRotationMatrix();
        }

        Eigen::Matrix4<Scalar> T() const {
            Eigen::Matrix4<Scalar> T = Eigen::Matrix4<Scalar>::Identity();
            T.template block<3, 3>(0, 0) = R();
            T.template block<3, 1>(0, 3) = t;
            return T;
        }
    };

}


#endif //ALG_SIM_ODOMETER_H
