#include "compute_features.hpp"
#include "compute_matches.hpp"
#include "config.h"
#include "SfMInit_image_listing.hpp"
#include "geometric_filter.hpp"
#include "global_SfM.hpp"
#include "pair_generator.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"
#include "openMVG/geometry/Similarity3.hpp"

// user defined, just for convenient
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
        std::ofstream camsFile(Config::outputDir + "/openMVG_Cameras.ply", std::ios::out);
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
        std::ofstream ptsFile(Config::outputDir + "/openMVG_Points.ply", std::ios::out);
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

int alignToFirstCamera(SfM_Data &sfmData) {
    Pose3 pose = (sfmData.poses[sfmData.GetViews().begin()->first]);
    double scale = 1.0;
    const geometry::Similarity3 sim(pose, scale);
    openMVG::sfm::ApplySimilarity(sim, sfmData);
    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
    // load the config file (yaml)
    Config::loadConfig("../config/config.yaml");

    // initialize the image list (e.g. the image's model, image path...)
    CALL_OPENMVG_FUNC(fMInitImageListing())
    // compute the features for each picture
    CALL_OPENMVG_FUNC(computeFeatures())
    // generate the pairs
    CALL_OPENMVG_FUNC(pairGenerator())
    // compute the matches
    CALL_OPENMVG_FUNC(computeMatches())
    // Geometric filtering of putative matches
    CALL_OPENMVG_FUNC(geometricFilter())

    // structure from the motion
    std::unique_ptr<ReconstructionEngine> sfm_engine;
    CALL_OPENMVG_FUNC(globalSfM(sfm_engine))
    auto sfmData = sfm_engine->Get_SfM_Data();

    // align
    CALL_OPENMVG_FUNC(alignToFirstCamera(sfmData))
    // after here, you can use the info of the sfmData
    // the structure of the sfmData is in the directory: docs/code_structure.drawio.png

    // write (for testing and visualization)
    CALL_OPENMVG_FUNC(writeSfMData(sfmData))

    return EXIT_SUCCESS;
}