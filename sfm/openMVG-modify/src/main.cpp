#include "compute_features.hpp"
#include "compute_matches.hpp"
#include "config.h"
#include "SfMInit_image_listing.hpp"
#include "geometric_filter.hpp"
#include "global_SfM.hpp"
#include "pair_generator.hpp"
#include "compute_SfM_data_color.hpp"

#define CALL_OPENMVG_FUNC(func)                                                         \
OPENMVG_LOG_INFO <<"\n------------------------------------------------"                 \
                 <<"\n"<<std::string(#func)                                             \
                 <<"\n------------------------------------------------";                \
if (func == EXIT_FAILURE) {                                                             \
    OPENMVG_LOG_ERROR <<"\n-----"<< std::string("[") + #func + "] failed.-----";        \
}else{                                                                                  \
    OPENMVG_LOG_INFO <<"\n-----"<< std::string("[") + #func + "] successfully.-----";   \
}

int writeSfMData(const SfM_Data &sfmData) {
    // write pose data
    {
        std::ofstream camsFile("../pose-points/openMVG_Cameras.ply", std::ios::out);
        if (!camsFile.is_open()) {
            OPENMVG_LOG_ERROR << "pose file open failed.";
            return EXIT_FAILURE;
        }

        IndexT view_with_pose_count = 0;
        for (const auto &view: sfmData.GetViews()) {
            view_with_pose_count += sfmData.IsPoseAndIntrinsicDefined(view.second.get());
        }
        camsFile << std::fixed << std::setprecision(15);
        camsFile << "ply                 " << std::endl <<
                 "format ascii 1.0    " << std::endl <<
                 "element vertex " << view_with_pose_count << std::endl <<
                 "property float qx    " << std::endl <<
                 "property float qy    " << std::endl <<
                 "property float qz    " << std::endl <<
                 "property float qw    " << std::endl <<
                 "property float tx    " << std::endl <<
                 "property float ty    " << std::endl <<
                 "property float tz    " << std::endl <<
                 "end_header          " << std::endl;
        for (const auto &view: sfmData.GetViews()) {
            // Export pose as Green points
            if (sfmData.IsPoseAndIntrinsicDefined(view.second.get())) {
                const geometry::Pose3 pose = sfmData.GetPoseOrDie(view.second.get());

                Eigen::Quaterniond q(pose.rotation());
                Eigen::Vector3d t = pose.translation();
                camsFile << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '
                         << t(0) << ' ' << t(1) << ' ' << t(2) << std::endl;
            }
        }
        camsFile.close();
    }
    // write points data
    {
        std::ofstream ptsFile("../pose-points/openMVG_Points.ply", std::ios::out);
        if (!ptsFile.is_open()) {
            OPENMVG_LOG_ERROR << "control points file open failed.";
            return EXIT_FAILURE;
        }
        const Landmarks &ctrPts = sfmData.GetControl_Points();
        const Landmarks &landmarks = sfmData.GetLandmarks();
        ptsFile << std::fixed << std::setprecision(15);
        ptsFile << "ply                 " << std::endl <<
                "format ascii 1.0    " << std::endl <<
                "element vertex " << ctrPts.size() + landmarks.size() << std::endl <<
                "property float x    " << std::endl <<
                "property float y    " << std::endl <<
                "property float z    " << std::endl <<
                "property uchar red  " << std::endl <<
                "property uchar green" << std::endl <<
                "property uchar blue " << std::endl <<
                "end_header          " << std::endl;
        for (const auto &ctrPt: ctrPts) {
            ptsFile << ctrPt.second.X(0) << ' '
                    << ctrPt.second.X(1) << ' '
                    << ctrPt.second.X(2) << ' '
                    << "255 0 0\n";
        }
        for (const auto &landmark: landmarks) {
            ptsFile << landmark.second.X(0) << ' '
                    << landmark.second.X(1) << ' '
                    << landmark.second.X(2) << ' '
                    << "0 255 0\n";
        }
        ptsFile.close();
    }
    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
    Config::loadConfig("../config/config.yaml");

    CALL_OPENMVG_FUNC(fMInitImageListing())
    CALL_OPENMVG_FUNC(computeFeatures())
    CALL_OPENMVG_FUNC(pairGenerator())
    CALL_OPENMVG_FUNC(computeMatches())
    CALL_OPENMVG_FUNC(geometricFilter())

    std::unique_ptr<ReconstructionEngine> sfm_engine;

    CALL_OPENMVG_FUNC(globalSfM(sfm_engine))
    CALL_OPENMVG_FUNC(computeSfMDataColor())

    auto sfmData = sfm_engine->Get_SfM_Data();

    /**
     * struct SfM_Data
     * {
     * /// Considered views
     * Views views;
     * /// Considered poses (indexed by view.id_pose)
     * Poses poses;
     * /// Considered camera intrinsics (indexed by view.id_intrinsic)
     * Intrinsics intrinsics;
     * /// Structure (3D points with their 2D observations)
     * Landmarks structure;
     * /// Controls points (stored as Landmarks (id_feat has no meaning here))
     * Landmarks control_points;
     *
     * /// Root Views path
     * std::string s_root_path;
     * };
     */
    CALL_OPENMVG_FUNC(writeSfMData(sfmData))

    return EXIT_SUCCESS;
}