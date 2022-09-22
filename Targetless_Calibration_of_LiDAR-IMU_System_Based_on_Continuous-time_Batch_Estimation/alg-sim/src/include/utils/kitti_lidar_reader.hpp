//
// Created by csl on 9/21/22.
//

#ifndef READ_LIDAR_FRAME_KITTI_LIDAR_READER_HPP
#define READ_LIDAR_FRAME_KITTI_LIDAR_READER_HPP

#include "fstream"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <vector>
#include <filesystem>
#include <algorithm>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

namespace ns_calib {

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
            auto floatSize = file.tellg() / sizeof(float);
            // count points' num
            auto pointSize = floatSize / 4;
            file.seekg(0, std::ios::beg);

            // read float buffer
            std::vector<float> lidarDataBuffer(floatSize);
            file.read(reinterpret_cast<char *>(&lidarDataBuffer[0]), floatSize * sizeof(float));
            file.close();

            // construct point cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            cloud->points.resize(pointSize);

            for (int i = 0; i < pointSize; ++i) {
                // assign
                cloud->points[i].x = lidarDataBuffer[i * 4 + 0];
                cloud->points[i].y = lidarDataBuffer[i * 4 + 1];
                cloud->points[i].z = lidarDataBuffer[i * 4 + 2];
                cloud->points[i].intensity = lidarDataBuffer[i * 4 + 3];
            }

            return cloud;
        }

        static std::vector<double> readFrameTimeStamp(const std::string &framePath) {
            using namespace boost::gregorian;
            using namespace boost::posix_time;

            std::vector<double> timeStamps;
            std::ifstream file(framePath);
            if (!file.is_open()) {
                return timeStamps;
            }
            std::string str;
            while (std::getline(file, str)) {
                auto strVec = splitString(str, ' ');
                unsigned int modJulianDay = from_string(strVec[0]).modjulian_day();

                auto hms = splitString(strVec[1], ':');
                unsigned int hour = stoi(hms[0]);
                unsigned int minute = stoi(hms[1]);
                double second = stod(hms[2]);

                double timeStamp = modJulianDay + ((hour * 60 + minute) * 60 + second) / 86400.0;
                timeStamps.push_back(timeStamp);
            }
            file.close();
            return timeStamps;
        }

        static std::vector<std::string> filesInDir(const std::string &directory) {
            std::vector<std::string> files;
            for (const auto &elem: std::filesystem::directory_iterator(directory))
                if (elem.status().type() != std::filesystem::file_type::directory)
                    files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
            std::sort(files.begin(), files.end());
            return files;
        }

    protected:
        static std::vector<std::string> splitString(const std::string &str, char splitor, bool ignoreEmpty = true) {
            std::vector<std::string> vec;
            auto iter = str.cbegin();
            while (true) {
                auto pos = std::find(iter, str.cend(), splitor);
                auto elem = std::string(iter, pos);
                if (!(elem.empty() && ignoreEmpty)) {
                    vec.push_back(elem);
                }
                if (pos == str.cend()) {
                    break;
                }
                iter = ++pos;
            }
            return vec;
        }

    };

}

#endif //READ_LIDAR_FRAME_KITTI_LIDAR_READER_HPP
