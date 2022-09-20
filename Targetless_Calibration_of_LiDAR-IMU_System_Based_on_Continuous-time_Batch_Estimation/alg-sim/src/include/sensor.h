//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_SENSOR_H
#define ALG_SIM_SENSOR_H

#include <ostream>
#include "opencv2/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/Core"

namespace ns_calib {
    template<class ScanType>
    class Observation {
    public:
        using Scan = ScanType;
        using ScanPtr = std::shared_ptr<Scan>;

    public:
        double timeStamp;
        ScanPtr scan;

    public:
        explicit Observation(double timeStamp, ScanPtr scan)
                : timeStamp(timeStamp), scan(scan) {}
    };

    template<class ObservationType, class ParameterType>
    class Sensor {
    public:
        using ObvTypePtr = std::shared_ptr<ObservationType>;
        using ParamTypePtr = std::shared_ptr<ParameterType>;

        using ObvType = ObservationType;
        using ParamType = ParameterType;

        using Ptr = std::shared_ptr<Sensor>;

    public:
        explicit Sensor() = default;

    public:
        ParamTypePtr param;
    };

    /**
     * LiDAR Sensor define
     */

    struct LiDARParameter {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned short rate{};
        unsigned short ringCount{};
        double noise{};
    };

    class LiDAR : public Sensor<Observation<pcl::PointCloud<pcl::PointXYZI>>, LiDARParameter> {
    public:
        using Ptr = std::shared_ptr<LiDAR>;

    public:
        LiDAR();
    };

    /**
     * IMU Sensor define
     */
    struct IMUScan {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d acce{};
        Eigen::Vector3d gyro{};
    };

    struct IMUParameter {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        struct {
            Eigen::Vector3d bias{};
            double noiseDensity{};
        } acce{};
        struct {
            Eigen::Vector3d bias{};
            double noiseDensity{};
        } gyro{};
        unsigned short rate{};
    };

    class IMU : public Sensor<Observation<IMUScan>, IMUParameter> {
    public:
        using Ptr = std::shared_ptr<IMU>;

    public:
        IMU();

    };

    /**
     * Camera Sensor define
     */
    struct PinholeCameraParameter {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned short rate{};
        Eigen::Matrix3d intrinsicsMat{};
        struct {
            Eigen::Vector3d radial{};
            Eigen::Vector2d tangential{};
        } distortion{};
    };

    class MonoPinholeCamera : public Sensor<Observation<cv::Mat>, PinholeCameraParameter> {
    public:
        using Ptr = std::shared_ptr<MonoPinholeCamera>;

    public:
        MonoPinholeCamera();

    };
}

#endif //ALG_SIM_SENSOR_H
