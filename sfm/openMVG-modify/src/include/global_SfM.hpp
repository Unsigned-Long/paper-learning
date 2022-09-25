//
// Created by csl on 9/24/22.
//

#ifndef SFM_GLOBAL_SFM_HPP
#define SFM_GLOBAL_SFM_HPP

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"

#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/types.hpp"

// SfM Engines
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM2.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerMaxPair.hpp"
#include "openMVG/sfm/pipelines/sequential/SfmSceneInitializerStellar.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;


enum class ESfMSceneInitializer {
    INITIALIZE_EXISTING_POSES,
    INITIALIZE_MAX_PAIR,
    INITIALIZE_AUTO_PAIR,
    INITIALIZE_STELLAR
};

enum class ESfMEngine {
    INCREMENTAL,
    INCREMENTALV2,
    GLOBAL
};

bool StringToEnum(const std::string &str, ESfMEngine &sfm_engine) {
    const std::map<std::string, ESfMEngine> string_to_enum_mapping = {
            {"INCREMENTAL",   ESfMEngine::INCREMENTAL},
            {"INCREMENTALV2", ESfMEngine::INCREMENTALV2},
            {"GLOBAL",        ESfMEngine::GLOBAL},
    };
    const auto it = string_to_enum_mapping.find(str);
    if (it == string_to_enum_mapping.end())
        return false;
    sfm_engine = it->second;
    return true;
}

bool StringToEnum(const std::string &str, ESfMSceneInitializer &scene_initializer) {
    const std::map<std::string, ESfMSceneInitializer> string_to_enum_mapping = {
            {"EXISTING_POSE", ESfMSceneInitializer::INITIALIZE_EXISTING_POSES},
            {"MAX_PAIR",      ESfMSceneInitializer::INITIALIZE_MAX_PAIR},
            {"AUTO_PAIR",     ESfMSceneInitializer::INITIALIZE_AUTO_PAIR},
            {"STELLAR",       ESfMSceneInitializer::INITIALIZE_STELLAR},
    };
    const auto it = string_to_enum_mapping.find(str);
    if (it == string_to_enum_mapping.end())
        return false;
    scene_initializer = it->second;
    return true;
}

/// From 2 given image filenames, find the two corresponding index in the View list
bool computeIndexFromImageNames(const SfM_Data &sfm_data,
                                const std::pair<std::string, std::string> &initialPairName,
                                Pair &initialPairIndex) {
    if (initialPairName.first == initialPairName.second) {
        OPENMVG_LOG_ERROR << "Invalid image names. You cannot use the same image to initialize a pair.";
        return false;
    }

    initialPairIndex = {UndefinedIndexT, UndefinedIndexT};

    /// List views filenames and find the one that correspond to the user ones:
    for (Views::const_iterator it = sfm_data.GetViews().begin(); it != sfm_data.GetViews().end(); ++it) {
        const View *v = it->second.get();
        const std::string filename = stlplus::filename_part(v->s_Img_path);
        if (filename == initialPairName.first) {
            initialPairIndex.first = v->id_view;
        } else {
            if (filename == initialPairName.second) {
                initialPairIndex.second = v->id_view;
            }
        }
    }
    return (initialPairIndex.first != UndefinedIndexT &&
            initialPairIndex.second != UndefinedIndexT);
}

int globalSfM(std::unique_ptr<ReconstructionEngine> &sfm_engine) {
    // Common options:
    std::string filename_sfm_data = Config::outputDir + "/matches/sfm_data.json";
    std::string directory_match;
    std::string filename_match = Config::outputDir + "/matches/matches.e.bin";
    std::string directory_output = Config::outputDir + "/reconstruction_global";
    std::string engine_name = Config::GlobalSfM::engineName;

    // Create output dir
    if (!stlplus::folder_exists(directory_output)) {
        if (!stlplus::folder_create(directory_output)) {
            OPENMVG_LOG_ERROR << "Cannot create reconstruction_dir directory";
            return EXIT_FAILURE;
        }
    }

    // Bundle adjustment options:
    std::string sIntrinsic_refinement_options = "ADJUST_ALL";
    std::string sExtrinsic_refinement_options = "ADJUST_ALL";
    bool b_use_motion_priors = false;

    // Incremental SfM options
    int triangulation_method = static_cast<int>(ETriangulationMethod::DEFAULT);
    int resection_method = static_cast<int>(resection::SolverType::DEFAULT);
    int user_camera_model = PINHOLE_CAMERA_RADIAL3;

    // SfM v1
    std::pair<std::string, std::string> initial_pair_string("", "");

    // SfM v2
    std::string sfm_initializer_method = "STELLAR";

    // Global SfM
    int rotation_averaging_method = int(ROTATION_AVERAGING_L2);
    int translation_averaging_method = int(TRANSLATION_AVERAGING_SOFTL1);

    b_use_motion_priors = false;

    // Check validity of command line parameters:
    if (!isValid(static_cast<ETriangulationMethod>(triangulation_method))) {
        OPENMVG_LOG_ERROR << "Invalid triangulation method";
        return EXIT_FAILURE;
    }

    if (!isValid(openMVG::cameras::EINTRINSIC(user_camera_model))) {
        OPENMVG_LOG_ERROR << "Invalid camera type";
        return EXIT_FAILURE;
    }

    const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
            cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
    if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0)) {
        OPENMVG_LOG_ERROR << "Invalid input for Bundle Adjustment Intrinsic parameter refinement option";
        return EXIT_FAILURE;
    }

    const sfm::Extrinsic_Parameter_Type extrinsic_refinement_options =
            sfm::StringTo_Extrinsic_Parameter_Type(sExtrinsic_refinement_options);
    if (extrinsic_refinement_options == static_cast<sfm::Extrinsic_Parameter_Type>(0)) {
        OPENMVG_LOG_ERROR << "Invalid input for the Bundle Adjustment Extrinsic parameter refinement option";
        return EXIT_FAILURE;
    }

    ESfMSceneInitializer scene_initializer_enum;
    if (!StringToEnum(sfm_initializer_method, scene_initializer_enum)) {
        OPENMVG_LOG_ERROR << "Invalid input for the SfM initializer option";
        return EXIT_FAILURE;
    }

    ESfMEngine sfm_engine_type;
    if (!StringToEnum(engine_name, sfm_engine_type)) {
        OPENMVG_LOG_ERROR << "Invalid input for the SfM Engine type";
        return EXIT_FAILURE;
    }

    if (rotation_averaging_method < ROTATION_AVERAGING_L1 ||
        rotation_averaging_method > ROTATION_AVERAGING_L2) {
        OPENMVG_LOG_ERROR << "Rotation averaging method is invalid";
        return EXIT_FAILURE;
    }

#ifndef USE_PATENTED_LIGT
    if (translation_averaging_method == TRANSLATION_LIGT) {
        OPENMVG_LOG_ERROR << "OpenMVG was not compiled with USE_PATENTED_LIGT cmake option";
        return EXIT_FAILURE;
    }
#endif
    if (translation_averaging_method < TRANSLATION_AVERAGING_L1 ||
        translation_averaging_method > TRANSLATION_LIGT) {
        OPENMVG_LOG_ERROR << "Translation averaging method is invalid";
        return EXIT_FAILURE;
    }

    if (directory_output.empty()) {
        OPENMVG_LOG_ERROR << "It is an invalid output directory";
        return EXIT_FAILURE;
    }

    // SfM related

    // Load input SfM_Data scene
    SfM_Data sfm_data;
    const ESfM_Data sfm_data_loading_etypes =
            scene_initializer_enum == ESfMSceneInitializer::INITIALIZE_EXISTING_POSES ?
            ESfM_Data(VIEWS | INTRINSICS | EXTRINSICS) : ESfM_Data(VIEWS | INTRINSICS);
    if (!Load(sfm_data, filename_sfm_data, sfm_data_loading_etypes)) {
        OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << filename_sfm_data << "\" cannot be read.";
        return EXIT_FAILURE;
    }

    if (!stlplus::folder_exists(directory_output)) {
        if (!stlplus::folder_create(directory_output)) {
            OPENMVG_LOG_ERROR << "Cannot create the output directory";
            return EXIT_FAILURE;
        }
    }

    //
    // Match and features
    //
    if (directory_match.empty() && !filename_match.empty() && stlplus::file_exists(filename_match)) {
        directory_match = stlplus::folder_part(filename_match);
        filename_match = stlplus::filename_part(filename_match);
    }

    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(directory_match, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type) {
        OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
        return EXIT_FAILURE;
    }

    // Features reading
    std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, directory_match, regions_type)) {
        OPENMVG_LOG_ERROR << "Cannot load view corresponding features in directory: " << directory_match << ".";
        return EXIT_FAILURE;
    }
    // Matches reading
    std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
    if (!(matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, filename_match)) ||
          matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.f.txt")) ||
          matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.f.bin")) ||
          matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.e.txt")) ||
          matches_provider->load(sfm_data, stlplus::create_filespec(directory_match, "matches.e.bin")))) {
        // Try to read the provided match filename or the default one (matches.f.txt/bin)
        OPENMVG_LOG_ERROR << "Cannot load the match file.";
        return EXIT_FAILURE;
    }

    std::unique_ptr<SfMSceneInitializer> scene_initializer;
    switch (scene_initializer_enum) {
        case ESfMSceneInitializer::INITIALIZE_AUTO_PAIR:
            OPENMVG_LOG_ERROR << "Not yet implemented.";
            return EXIT_FAILURE;
            break;
        case ESfMSceneInitializer::INITIALIZE_MAX_PAIR:
            scene_initializer = std::make_unique<SfMSceneInitializerMaxPair>(
                    sfm_data,
                    feats_provider.get(),
                    matches_provider.get()
            );
            break;
        case ESfMSceneInitializer::INITIALIZE_EXISTING_POSES:
            scene_initializer = std::make_unique<SfMSceneInitializer>(
                    sfm_data,
                    feats_provider.get(),
                    matches_provider.get()
            );
            break;
        case ESfMSceneInitializer::INITIALIZE_STELLAR:
            scene_initializer = std::make_unique<SfMSceneInitializerStellar>(
                    sfm_data,
                    feats_provider.get(),
                    matches_provider.get()
            );
            break;
        default:
            OPENMVG_LOG_ERROR << "Unknown SFM Scene initializer method";
            return EXIT_FAILURE;
    }
    if (!scene_initializer) {
        OPENMVG_LOG_ERROR << "Invalid scene initializer.";
        return EXIT_FAILURE;
    }

    switch (sfm_engine_type) {
        case ESfMEngine::INCREMENTAL: {
            auto *engine = new SequentialSfMReconstructionEngine(
                    sfm_data,
                    directory_output,
                    stlplus::create_filespec(directory_output, "Reconstruction_Report.html")
            );

            // Configuration:
            engine->SetFeaturesProvider(feats_provider.get());
            engine->SetMatchesProvider(matches_provider.get());

            // Configure reconstruction parameters
            engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
            engine->Set_Use_Motion_Prior(b_use_motion_priors);
            engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
            engine->SetResectionMethod(static_cast<resection::SolverType>(resection_method));

            // Handle Initial pair parameter
            if (!initial_pair_string.first.empty() && !initial_pair_string.second.empty()) {
                Pair initial_pair_index;
                if (!computeIndexFromImageNames(sfm_data, initial_pair_string, initial_pair_index)) {
                    OPENMVG_LOG_ERROR << "Could not find the initial pairs <" << initial_pair_string.first
                                      << ", " << initial_pair_string.second << ">!";
                    return EXIT_FAILURE;
                }
                engine->setInitialPair(initial_pair_index);
            }

            sfm_engine.reset(engine);
        }
            break;
        case ESfMEngine::INCREMENTALV2: {
            auto *engine = new SequentialSfMReconstructionEngine2(
                    scene_initializer.get(),
                    sfm_data,
                    directory_output,
                    stlplus::create_filespec(directory_output, "Reconstruction_Report.html")
            );

            // Configuration:
            engine->SetFeaturesProvider(feats_provider.get());
            engine->SetMatchesProvider(matches_provider.get());

            // Configure reconstruction parameters
            engine->Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
            engine->SetUnknownCameraType(EINTRINSIC(user_camera_model));
            engine->Set_Use_Motion_Prior(b_use_motion_priors);
            engine->SetTriangulationMethod(static_cast<ETriangulationMethod>(triangulation_method));
            engine->SetResectionMethod(static_cast<resection::SolverType>(resection_method));

            sfm_engine.reset(engine);
        }
            break;
        case ESfMEngine::GLOBAL: {
            auto *engine = new GlobalSfMReconstructionEngine_RelativeMotions(
                    sfm_data,
                    directory_output,
                    stlplus::create_filespec(directory_output, "Reconstruction_Report.html")
            );

            // Configuration:
            engine->SetFeaturesProvider(feats_provider.get());
            engine->SetMatchesProvider(matches_provider.get());

            // Configure reconstruction parameters
            engine->Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
            engine->Set_Use_Motion_Prior(b_use_motion_priors);

            // Configure motion averaging method
            engine->SetRotationAveragingMethod(ERotationAveragingMethod(rotation_averaging_method));
            engine->SetTranslationAveragingMethod(ETranslationAveragingMethod(translation_averaging_method));

            sfm_engine.reset(engine);
        }
            break;
        default:
            break;
    }
    if (!sfm_engine) {
        OPENMVG_LOG_ERROR << "Cannot create the requested SfM Engine.";
        return EXIT_FAILURE;
    }

    sfm_engine->Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
    sfm_engine->Set_Extrinsics_Refinement_Type(extrinsic_refinement_options);

    //---------------------------------------
    // Sequential reconstruction process
    //---------------------------------------

    openMVG::system::Timer timer;

    if (sfm_engine->Process()) {
        OPENMVG_LOG_INFO << " Total Sfm took (s): " << timer.elapsed();

        OPENMVG_LOG_INFO << "...Generating SfM_Report.html";
        Generate_SfM_Report(sfm_engine->Get_SfM_Data(),
                            stlplus::create_filespec(directory_output, "SfMReconstruction_Report.html"));

        //-- Export to disk computed scene (data & viewable results)
        OPENMVG_LOG_INFO << "...Export SfM_Data to disk.";
        Save(
                sfm_engine->Get_SfM_Data(),
                stlplus::create_filespec(directory_output, "sfm_data", ".bin"),
                ESfM_Data(ALL)
        );

        Save(
                sfm_engine->Get_SfM_Data(),
                stlplus::create_filespec(directory_output, "cloud_and_poses", ".ply"),
                ESfM_Data(ALL)
        );

        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}


#endif //SFM_GLOBAL_SFM_HPP
