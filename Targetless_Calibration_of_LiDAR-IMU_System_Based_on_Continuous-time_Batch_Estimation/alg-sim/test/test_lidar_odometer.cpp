#include "odometer/lidar_odometer.h"
#include "utils/kitti_lidar_reader.hpp"
#include "pcl/visualization/cloud_viewer.h"
#include "thread"
#include "artwork/logger/logger.h"
#include "utils/status.hpp"

int main(int argc, char **argv) {
    using namespace ns_calib;

    try {
        // load configure file
        if (!Config::initConfig("../config/config.yaml")) {
            throw Status(Status::Flag::ERROR, "Initialize Configure Failed in 'Config::initConfig'.");
        }
        // lidar odometer
        auto lidarOdometer = LiDAROdometer::create(ns_calib::LiDAR(ns_calib::LiDARParameter()));

        // load lidar frames
        auto lidarFrames =
                KittiReader::filesInDir("/home/csl/dataset/kitti/data/2011_09_26_drive_0001_sync"
                                        "/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data");

        // visualization
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

        // run visualization in another thread
        std::thread thread([&viewer]() { while (!viewer.wasStopped()) {}});

        // feed frame
        for (const auto &framePath: lidarFrames) {
            if (viewer.wasStopped()) {
                break;
            }
            auto cloud = KittiReader::readFrame(framePath);
            LOG_VAR(framePath);

            lidarOdometer->feedObservation(
                    std::make_shared<LiDAROdometer::Obv>(0, cloud),
                    LiDAROdometer::Pose(), false
            );

            viewer.showCloud(lidarOdometer->getMap()->frameData);
        }
        thread.join();

    } catch (const Status &status) {
        std::cout << status << std::endl;
    }
    return 0;
}