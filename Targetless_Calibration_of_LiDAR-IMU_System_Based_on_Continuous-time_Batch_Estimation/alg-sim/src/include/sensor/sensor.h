//
// Created by csl on 9/20/22.
//

#ifndef ALG_SIM_SENSOR_H
#define ALG_SIM_SENSOR_H

#include <ostream>
#include "utils/enum_cast.hpp"
#include "opencv2/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/Core"

namespace ns_calib {
    template<class FrameDataType>
    class Observation {
    public:
        using FrameData = FrameDataType;
        using FrameDataPtr = std::shared_ptr<FrameData>;

    public:
        double timeStamp;
        FrameDataPtr frameData;

    public:
        explicit Observation(double timeStamp, FrameDataPtr frameDataPtr)
                : timeStamp(timeStamp), frameData(frameDataPtr) {}
    };

    enum class SensorType {
        CAMERA_PINHOLE = 0, LiDAR_3D = 1, IMU_MEMS = 2
    };

    template<class FrameDataType, class ParameterType>
    class Sensor {
    public:
        using Obv = Observation<FrameDataType>;
        using ObvPtr = std::shared_ptr<Obv>;

        using FrameData = typename Obv::FrameData;
        using FrameDataPtr = typename Obv::FrameDataPtr;

        using Param = ParameterType;
        using ParamPtr = std::shared_ptr<Param>;

        using Ptr = std::shared_ptr<Sensor>;

    public:
        explicit Sensor(Param param) : param(param) {}

        /**
         * the type of this sensor
         */
        [[nodiscard]] virtual SensorType type() const = 0;

        friend std::ostream &operator<<(std::ostream &os, const Sensor &sensor) {
            os << "type: " << EnumCast::enumToString(sensor.type()) << ", param: " << sensor.param;
            return os;
        }

    public:
        Param param;
    };

    /**
     * LiDAR Sensor define
     */
    struct LiDARParameter {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned short frameRate{};
        unsigned short ringCount{};
        double noise{};

        float HorizontalFOV{};
        float verticalFOV{};
        float maxDistance{};
    };

    class LiDAR : public Sensor<pcl::PointCloud<pcl::PointXYZI>, LiDARParameter> {
    public:
        using Ptr = std::shared_ptr<LiDAR>;

    public:
        explicit LiDAR(const Param &param) : Sensor(param) {}

        [[nodiscard]] SensorType type() const override;
    };

    /**
     * IMU Sensor define
     */
    struct IMUFrameData {
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
        unsigned short frameRate{};
    };

    class IMU : public Sensor<IMUFrameData, IMUParameter> {
    public:
        using Ptr = std::shared_ptr<IMU>;

    public:
        explicit IMU(const Param &param) : Sensor(param) {}

        [[nodiscard]] SensorType type() const override;
    };

    /**
     * Camera Sensor define
     */
    struct PinholeCameraParameter {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        unsigned short frameRate{};
        Eigen::Matrix3d intrinsicsMat{};
        struct {
            Eigen::Vector3d radial{};
            Eigen::Vector2d tangential{};
        } distortion{};
    };

    class MonoPinholeCamera : public Sensor<cv::Mat, PinholeCameraParameter> {
    public:
        using Ptr = std::shared_ptr<MonoPinholeCamera>;

    public:
        explicit MonoPinholeCamera(const Param &param) : Sensor(param) {}

        [[nodiscard]] SensorType type() const override;
    };
}

#endif //ALG_SIM_SENSOR_H
