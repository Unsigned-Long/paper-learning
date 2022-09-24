//
// Created by csl on 9/23/22.
//

#include "artwork/logger/logger.h"
#include "fstream"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "exception"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/common/transforms.h>
#include "thread"
#include "chrono"

struct Reader {
    static std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>>
    readCameras(const std::string &filePath) {
        std::fstream file(filePath, std::ios::in);
        if (!file.is_open()) {
            throw std::runtime_error("[Reader::readCameras] open file failed.");
        }

        std::string strLine;

        std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> pose;
        {
            // ply
            std::getline(file, strLine);
            // format
            std::getline(file, strLine);
            // element vertex num
            std::getline(file, strLine);
            auto num = std::stoul(split(strLine, ' ')[2]);
            pose.resize(num);
            // end header
            while (getline(file, strLine)) {
                if (strLine.size() >= 10 && strLine.substr(0, 10) == "end_header") {
                    break;
                }
            }
        }
        for (auto &[q, t]: pose) {
            getline(file, strLine);
            auto strVec = split(strLine, ' ');

            q.x() = std::stof(strVec[0]);
            q.y() = std::stof(strVec[1]);
            q.z() = std::stof(strVec[2]);
            q.w() = std::stof(strVec[3]);
            q.normalize();

            t(0) = std::stof(strVec[4]);
            t(1) = std::stof(strVec[5]);
            t(2) = std::stof(strVec[6]);
        }
        file.close();
        return pose;
    }

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    readPoints(const std::string &filePath) {
        std::fstream file(filePath, std::ios::in);
        if (!file.is_open()) {
            throw std::runtime_error("[Reader::readPoints] open file failed.");
        }
        std::string strLine;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        {
            // ply
            std::getline(file, strLine);
            // format
            std::getline(file, strLine);
            // element vertex num
            std::getline(file, strLine);
            auto ptsNum = std::stoul(split(strLine, ' ')[2]);
            cloud->resize(ptsNum);
            // end header
            while (getline(file, strLine)) {
                if (strLine.size() >= 10 && strLine.substr(0, 10) == "end_header") {
                    break;
                }
            }
        }
        for (auto &p: *cloud) {
            getline(file, strLine);
            auto strVec = split(strLine, ' ');

            p.x = std::stof(strVec[0]);
            p.y = std::stof(strVec[1]);
            p.z = std::stof(strVec[2]);
            p.r = std::stoi(strVec[3]);
            p.g = std::stoi(strVec[4]);
            p.b = std::stoi(strVec[5]);
        }
        file.close();
        return cloud;
    }

protected:
    static std::vector<std::string>
    split(const std::string &str, char splitor, bool ignoreEmpty = true) {
        std::vector<std::string> vec;
        std::string elem;
        std::stringstream stream;
        stream << str;
        while (std::getline(stream, elem, splitor)) {
            if (elem.empty() && ignoreEmpty) {
                continue;
            }
            vec.push_back(elem);
        }
        return vec;
    }

};

void visualization(const std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> &cameras,
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points) {
    pcl::visualization::PCLVisualizer viewer("ViewSpace");

    // coordinates
    viewer.addCoordinateSystem(0.2, "Origin");

    viewer.addPointCloud(points, "points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");

    Eigen::Vector4f ptsCenter;
    pcl::compute3DCentroid(*points, ptsCenter);

    Eigen::Vector3f camCenter(0.0, 0.0, 0.0);
    Eigen::Vector3f camUp(0.0, 0.0, 0.0);

    for (int i = 0; i < cameras.size(); ++i) {
        // from world to camera
        const auto &q_cw = cameras[i].first;
        const auto &t_cw = cameras[i].second;

        Eigen::Isometry3f Tcw(q_cw);
        Tcw.pretranslate(t_cw);
        Eigen::Isometry3f Twc = Tcw.inverse();

        viewer.addCoordinateSystem(0.05f, Eigen::Affine3f(Twc.cast<float>().affine()),
                                   "Camera" + std::to_string(i));

        camCenter += Twc * Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        // vector, without translation
        camUp += Twc.rotation() * Eigen::Vector3f(0.0f, -0.05f, 0.0f);

        Eigen::Vector3f p1 = Twc * Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f p2 = Twc * Eigen::Vector3f(0.0f, 0.0f, 0.05f);

        // Cone
        pcl::ModelCoefficients coneCoeff;
        coneCoeff.values.resize(7); // We need 7 values
        coneCoeff.values[0] = p1(0);
        coneCoeff.values[1] = p1(1);
        coneCoeff.values[2] = p1(2);
        coneCoeff.values[3] = p2(0) - p1(0);
        coneCoeff.values[4] = p2(1) - p1(1);
        coneCoeff.values[5] = p2(2) - p1(2);
        coneCoeff.values[6] = 30.0; // degrees
        viewer.addCone(coneCoeff, "Cone" + std::to_string(i));

    }

    camCenter /= cameras.size();
    camUp /= cameras.size();
//    camCenter *= 2.5;
    viewer.setCameraPosition(camCenter(0), camCenter(1), camCenter(2),
                             ptsCenter(0), ptsCenter(1), ptsCenter(2),
                             camUp(0), camUp(1), camUp(2));

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }
}

int main(int argc, char **argv) {
    auto cloud = Reader::readPoints("../output/_points.ply");
    auto cameras = Reader::readCameras("../output/_cameras.ply");
    visualization(cameras, cloud);
    return 0;
}