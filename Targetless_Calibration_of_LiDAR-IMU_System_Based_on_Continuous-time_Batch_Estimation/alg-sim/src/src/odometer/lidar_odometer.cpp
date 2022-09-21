//
// Created by csl on 9/21/22.
//

#include "odometer/lidar_odometer.h"
#include "sensor/sensor.h"

namespace ns_calib {

    /*
     * LiDAROdometer
     */
    LiDAROdometer::LiDAROdometer(const LiDAR &sensor)
            : Odometer(sensor), map(nullptr),
              rawObvVec(), poseVec_frame_to_map(), initialized(false),
              filter(nullptr), ndt(nullptr) {}

    LiDAROdometer::Ptr LiDAROdometer::create(const LiDAR &sensor) {
        return std::make_shared<ns_calib::LiDAROdometer>(sensor);
    }

    bool LiDAROdometer::feedObservation(const Sensor::ObvPtr &obv,
                                        const Pose &initPose_cur_to_map,
                                        bool updateMap) {
        this->rawObvVec.push_back(obv);

        if (!this->initialized) {
            this->map = obv;
            this->initAVGFilter();
            this->initNDT();
            return true;
        }

        auto filteredInputCloud = filterFrameData(obv->frameData);

        // Setting point cloud to be aligned.
        ndt->setInputSource(filteredInputCloud);
        // Setting point cloud to be aligned to.
        ndt->setInputTarget(map->frameData);

        // Calculating required rigid transform to align the input cloud to the target cloud.
        pcl::PointCloud<Point>::Ptr alignedCloud(new pcl::PointCloud<Point>);
        ndt->align(*alignedCloud, initPose_cur_to_map.T().cast<float>());

        Eigen::Matrix4d pose_frame_map = ndt->getFinalTransformation().cast<double>();

        this->poseVec_frame_to_map.emplace_back(
                Pose::Rotation(pose_frame_map.block<3, 3>(0, 0)),
                Pose::Translation(pose_frame_map.block<3, 1>(0, 3))
        );

        if (updateMap) {
            *(this->map->frameData) += *(alignedCloud);
        }

        return true;
    }

    void LiDAROdometer::initAVGFilter() {
        this->filter = std::make_shared<pcl::ApproximateVoxelGrid<Point>>();
        this->filter->setLeafSize(Config::LiDAROdometer::AVGFilter::LEAF_SIZE);
    }

    LiDAROdometer::FrameDataPtr LiDAROdometer::filterFrameData(const FrameDataPtr &input) {
        FrameDataPtr filteredCloud(new FrameData);
        this->filter->setInputCloud(input);
        this->filter->filter(*filteredCloud);
        return filteredCloud;
    }

    void LiDAROdometer::initNDT() {
        this->ndt = std::make_shared<pcl::NormalDistributionsTransform<Point, Point>>();

        // Setting minimum transformation difference for termination condition.
        ndt->setTransformationEpsilon(Config::LiDAROdometer::NDT::TRANS_EPSILON);
        // Setting maximum step size for More-Thuente line search.
        ndt->setStepSize(Config::LiDAROdometer::NDT::STEP_SIZE);
        // Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt->setResolution(Config::LiDAROdometer::NDT::RESOLUTION);
        // Setting max number of registration iterations.
        ndt->setMaximumIterations(Config::LiDAROdometer::NDT::MAX_ITERATIONS);
    }

}