//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_ODOMETER_H
#define ALG_SIM_ODOMETER_H

#include "sensor/sensor.h"
#include "config.h"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

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

    template<class ScalarType>
    struct OdometerPose {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Scale = ScalarType;
        using Rotation = Sophus::SO3<Scale>;
        using Translation = Eigen::Vector3<Scale>;

        Rotation so3;
        Translation t;
        double timeStamp;

        OdometerPose(const Rotation &so3, const Translation &t, double timeStamp)
                : so3(so3), t(t), timeStamp(timeStamp) {}

        OdometerPose(const Rotation &r, const Translation &t)
                : so3(r), t(t), timeStamp() {}

        OdometerPose() : so3(), t(Translation::Zero()), timeStamp() {}

        Eigen::Quaternion<ScalarType> q() const {
            return so3.unit_quaternion();
        }

        Eigen::Matrix3<ScalarType> R() const {
            return q().toRotationMatrix();
        }

        Sophus::SE3<ScalarType> se3() const {
            return Sophus::SE3<ScalarType>(so3, t);
        }

        Eigen::Matrix4<ScalarType> T() const {
            Eigen::Matrix4<ScalarType> T = Eigen::Matrix4<ScalarType>::Identity();
            T.template block<3, 3>(0, 0) = R();
            T.template block<3, 1>(0, 3) = t;
            return T;
        }

        static OdometerPose fromT(const Eigen::Matrix4<ScalarType> &T) {
            Eigen::Matrix3<ScalarType> rotMat = T.template block<3, 3>(0, 0);
            rotMat = adjustRotationMatrix(rotMat);

            OdometerPose pose;
            pose.so3 = Rotation(rotMat);
            pose.t = T.template block<3, 1>(0, 3);
            return pose;
        }

        static OdometerPose fromRt(const Eigen::Matrix3<ScalarType> &R, const Eigen::Vector3<ScalarType> &t) {
            Eigen::Matrix3<ScalarType> rotMat = adjustRotationMatrix(R);

            OdometerPose pose;
            pose.so3 = Rotation(rotMat);
            pose.t = t;
            return pose;
        }

    protected:
        static Eigen::Matrix3<ScalarType> adjustRotationMatrix(const Eigen::Matrix3<ScalarType> &rotMat) {
            // adjust
            Eigen::JacobiSVD<Eigen::Matrix3<ScalarType>> svd(rotMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
            const Eigen::Matrix3<ScalarType> &vMatrix = svd.matrixV();
            const Eigen::Matrix3<ScalarType> &uMatrix = svd.matrixU();
            Eigen::Matrix3<ScalarType> adjustedRotMat = uMatrix * vMatrix.transpose();
            return adjustedRotMat;
        }
    };

}


#endif //ALG_SIM_ODOMETER_H
