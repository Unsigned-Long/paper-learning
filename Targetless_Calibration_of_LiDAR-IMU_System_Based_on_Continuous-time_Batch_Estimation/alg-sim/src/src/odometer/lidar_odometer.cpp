//
// Created by csl on 9/21/22.
//

#include "odometer/lidar_odometer.h"
#include "sensor/sensor.h"
#include "artwork/logger/logger.h"

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
                                        const Pose &posePred_cur_to_last,
                                        bool updateMap) {
        this->rawObvVec.push_back(obv);


        if (!this->initialized) {
            this->map = obv;
            this->poseVec_frame_to_map.emplace_back(Pose());
            this->initAVGFilter();
            this->initNDT();
            initialized = true;
            return true;
        }

        auto filteredInputCloud = filterFrameData(obv->frameData);

        // Setting point cloud to be aligned.
        ndt->setInputSource(filteredInputCloud);
        // Setting point cloud to be aligned to.
        ndt->setInputTarget(map->frameData);

        // Calculating required rigid transform to align the input cloud to the target cloud.
        pcl::PointCloud<Point>::Ptr alignedCloud(new pcl::PointCloud<Point>);
        ndt->align(
                *alignedCloud,
                this->poseVec_frame_to_map.back().T().cast<float>() *
                posePred_cur_to_last.T().cast<float>()
        );

        Eigen::Matrix4d pose_frame_to_map = ndt->getFinalTransformation().cast<double>();
        this->poseVec_frame_to_map.emplace_back(Pose::fromT(pose_frame_to_map));

        LOG_VAR(pose_frame_to_map);
        LOG_VAR(filteredInputCloud->front(), alignedCloud->front());

        if (updateMap) {
            pcl::PointCloud<Point>::Ptr scan_in_target(new pcl::PointCloud<Point>());
            pcl::transformPointCloud(*filteredInputCloud, *scan_in_target, pose_frame_to_map);
            *(this->map->frameData) += *(scan_in_target);
            this->map->frameData = filteredInputCloud;
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

    const LiDAROdometer::ObvPtr &LiDAROdometer::getMap() const {
        return map;
    }

}