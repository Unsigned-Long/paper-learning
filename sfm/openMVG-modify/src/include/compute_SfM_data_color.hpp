//
// Created by csl on 9/24/22.
//

#ifndef SFM_COMPUTE_SFM_DATA_COLOR_HPP
#define SFM_COMPUTE_SFM_DATA_COLOR_HPP

// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_colorization.hpp"
#include "openMVG/system/logger.hpp"
#include "openMVG/types.hpp"

#include "SfM_ply_helper.hpp"
#include "third_party/cmdLine/cmdLine.h"


using namespace openMVG;
using namespace openMVG::sfm;

/// Export camera poses positions as a Vec3 vector
void GetCameraPositions(const SfM_Data &sfm_data, std::vector<Vec3> &vec_camPosition) {
    for (const auto &view: sfm_data.GetViews()) {
        if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get())) {
            const geometry::Pose3 pose = sfm_data.GetPoseOrDie(view.second.get());
            vec_camPosition.push_back(pose.center());
        }
    }
}

// Convert from a SfM_Data format to another
int computeSfMDataColor() {

    std::string
            sSfM_Data_Filename_In = Config::outputDir + "/reconstruction_global/sfm_data.bin",
            sOutputPLY_Out = Config::outputDir + "/reconstruction_global/colorized.ply";

    if (sOutputPLY_Out.empty()) {
        OPENMVG_LOG_ERROR << "No output PLY filename specified.";
        return EXIT_FAILURE;
    }

    // Load input SfM_Data scene
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL))) {
        OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read.";
        return EXIT_FAILURE;
    }

    // Compute the scene structure color
    std::vector<Vec3> vec_3dPoints, vec_tracksColor, vec_camPosition;
    if (ColorizeTracks(sfm_data, vec_3dPoints, vec_tracksColor)) {
        GetCameraPositions(sfm_data, vec_camPosition);

        // Export the SfM_Data scene in the expected format
        if (plyHelper::exportToPly(vec_3dPoints, vec_camPosition, sOutputPLY_Out, &vec_tracksColor)) {
            return EXIT_SUCCESS;
        }
    }

    return EXIT_FAILURE;
}


#endif //SFM_COMPUTE_SFM_DATA_COLOR_HPP
