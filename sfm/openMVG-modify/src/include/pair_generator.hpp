//
// Created by csl on 9/24/22.
//

#ifndef SFM_PAIR_GENERATOR_HPP
#define SFM_PAIR_GENERATOR_HPP

// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2019 Romuald PERROT

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>
#include "config.h"

/**
 * @brief Current list of available pair mode
 *
 */
enum EPairMode {
    PAIR_EXHAUSTIVE = 0, // Build every combination of image pairs
    PAIR_CONTIGUOUS = 1  // Only consecutive image pairs (useful for video mode)
};

using namespace openMVG;
using namespace openMVG::sfm;

// This executable computes pairs of images to be matched
int pairGenerator() {

    std::string sSfMDataFilename = Config::outputDir + "/matches/sfm_data.json";
    std::string sOutputPairsFilename = Config::outputDir + "/matches/pairs.bin";
    std::string sPairMode = "EXHAUSTIVE";
    int iContiguousCount = -1;

    // 0. Parse parameters
    OPENMVG_LOG_INFO << "--input_file       : " << sSfMDataFilename << "\n"
                     << "--output_file      : " << sOutputPairsFilename << "\n"
                     << "Optional parameters\n"
                     << "--pair_mode        : " << sPairMode << "\n"
                     << "--contiguous_count : " << iContiguousCount << "\n"
                     << std::endl;

    if (sSfMDataFilename.empty()) {
        std::cerr << "[Error] Input file not set." << std::endl;
        exit(EXIT_FAILURE);
    }
    if (sOutputPairsFilename.empty()) {
        std::cerr << "[Error] Output file not set." << std::endl;
        exit(EXIT_FAILURE);
    }

    EPairMode pairMode;
    if (sPairMode == "EXHAUSTIVE") {
        pairMode = PAIR_EXHAUSTIVE;
    } else if (sPairMode == "CONTIGUOUS") {
        if (iContiguousCount == -1) {
            std::cerr << "[Error] Contiguous pair mode selected but contiguous_count not set." << std::endl;
            exit(EXIT_FAILURE);
        }

        pairMode = PAIR_CONTIGUOUS;
    }

    // 1. Load SfM data scene
    std::cout << "Loading scene.";
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfMDataFilename, ESfM_Data(VIEWS | INTRINSICS))) {
        std::cerr << std::endl
                  << "The input SfM_Data file \"" << sSfMDataFilename << "\" cannot be read." << std::endl;
        exit(EXIT_FAILURE);
    }
    const size_t NImage = sfm_data.GetViews().size();

    // 2. Compute pairs
    std::cout << "Computing pairs." << std::endl;
    Pair_Set pairs;
    switch (pairMode) {
        case PAIR_EXHAUSTIVE: {
            pairs = exhaustivePairs(NImage);
            break;
        }
        case PAIR_CONTIGUOUS: {
            pairs = contiguousWithOverlap(NImage, iContiguousCount);
            break;
        }
        default: {
            std::cerr << "Unknown pair mode" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // 3. Save pairs
    std::cout << "Saving pairs." << std::endl;
    if (!savePairs(sOutputPairsFilename, pairs)) {
        std::cerr << "Failed to save pairs to file: \"" << sOutputPairsFilename << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}


#endif //SFM_PAIR_GENERATOR_HPP
