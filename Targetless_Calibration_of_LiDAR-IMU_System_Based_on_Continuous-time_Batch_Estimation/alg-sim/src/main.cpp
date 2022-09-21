#include "calib.h"
#include "odometer/lidar_odometer.h"
#include "odometer/visual_odometer.h"
#include "imu_bspline.h"
#include "fstream"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <filesystem>
#include <algorithm>
#include "pcl/visualization/cloud_viewer.h"
#include "thread"
#include "artwork/logger/logger.h"

struct KittiReader {
public:
    static pcl::PointCloud<pcl::PointXYZI>::Ptr readFrame(const std::string &framePath) {
        // open binary file
        std::ifstream file(framePath, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            return nullptr;
        }
        // count the float values' num
        file.seekg(0, std::ios::end);
        long floatSize = file.tellg() / sizeof(float);
        // count points' num
        auto pointSize = floatSize / 4;
        file.seekg(0, std::ios::beg);

        // read float buffer
        std::vector<float> lidarDataBuffer(floatSize);
        file.read(reinterpret_cast<char *>(&lidarDataBuffer[0]), floatSize * sizeof(float));
        file.close();

        // construct point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->points.reserve(pointSize);

        for (int i = 0; i < pointSize; ++i) {
            float x = lidarDataBuffer[i * 4 + 0];
            float y = lidarDataBuffer[i * 4 + 1];
            float z = lidarDataBuffer[i * 4 + 2];
            float intensity = lidarDataBuffer[i * 4 + 3];
            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(intensity)) {
                continue;
            }
            // emplace_back
            cloud->emplace_back(x, y, z, intensity);
        }
        cloud->points.shrink_to_fit();

        return cloud;
    }

    static std::vector<std::string> filesInDir(const std::string &directory) {
        std::vector<std::string> files;
        for (const auto &elem: std::filesystem::directory_iterator(directory))
            if (elem.status().type() != std::filesystem::file_type::directory)
                files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
        std::sort(files.begin(), files.end());
        return files;
    }
};

int main(int argc, char **argv) {
    try {
        // load configure file
        if (!ns_calib::Config::initConfig("../config/config.yaml")) {
            throw ns_calib::Status(
                    ns_calib::Status::Flag::ERROR,
                    "Initialize Configure Failed in 'Config::initConfig'."
            );
        }
        // lidar odometer
        auto lidarOdometer = ns_calib::LiDAROdometer::create(ns_calib::LiDAR(ns_calib::LiDARParameter()));

        // load lidar frames
        auto lidarFrames = KittiReader::filesInDir("/home/csl/dataset/kitti/data/2011_09_26_drive_0001_sync"
                                                   "/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data");

        // visualization
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

        // run visualization in another thread
        std::thread thread([&viewer]() {
            while (!viewer.wasStopped()) {}
        });

        // feed frame
        for (const auto &framePath: lidarFrames) {
            auto cloud = KittiReader::readFrame(framePath);
            LOG_VAR(framePath);

            lidarOdometer->feedObservation(
                    std::make_shared<ns_calib::LiDAROdometer::Obv>(0, cloud)
            );

            viewer.showCloud(lidarOdometer->getMap()->frameData);
//            std::cin.get();
        }
        thread.join();

    } catch (const ns_calib::Status &status) {
        std::cout << status << std::endl;
    }
    return 0;
}