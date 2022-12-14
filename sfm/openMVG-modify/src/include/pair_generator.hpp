//
// Created by csl on 9/24/22.
//

#ifndef SFM_PAIR_GENERATOR_HPP
#define SFM_PAIR_GENERATOR_HPP

#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iostream>
#include "config.h"

/**
 * @brief Current list of available pair mode
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
    std::string sPairMode = Config::PairGenerator::pairMode;
    int iContiguousCount = Config::PairGenerator::contiguousCount;

    // 0. Parse parameters
    OPENMVG_LOG_INFO << "--input_file       : " << sSfMDataFilename << "\n"
                     << "--output_file      : " << sOutputPairsFilename << "\n"
                     << "Optional parameters\n"
                     << "--pair_mode        : " << sPairMode << "\n"
                     << "--contiguous_count : " << iContiguousCount << "\n";

    if (sSfMDataFilename.empty()) {
        OPENMVG_LOG_ERROR << "[Error] Input file not set.";
        exit(EXIT_FAILURE);
    }
    if (sOutputPairsFilename.empty()) {
        OPENMVG_LOG_ERROR << "[Error] Output file not set.";
        exit(EXIT_FAILURE);
    }

    EPairMode pairMode;
    if (sPairMode == "EXHAUSTIVE") {
        pairMode = PAIR_EXHAUSTIVE;
    } else if (sPairMode == "CONTIGUOUS") {
        if (iContiguousCount == -1) {
            OPENMVG_LOG_ERROR << "[Error] Contiguous pair mode selected but contiguous_count not set.";
            exit(EXIT_FAILURE);
        }

        pairMode = PAIR_CONTIGUOUS;
    }

    // 1. Load SfM data scene
    OPENMVG_LOG_INFO << "Loading scene.";
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfMDataFilename, ESfM_Data(VIEWS | INTRINSICS))) {
        OPENMVG_LOG_ERROR << std::endl
                          << "The input SfM_Data file \"" << sSfMDataFilename << "\" cannot be read.";
        exit(EXIT_FAILURE);
    }
    const size_t NImage = sfm_data.GetViews().size();

    // 2. Compute pairs
    OPENMVG_LOG_INFO << "Computing pairs.";
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
            OPENMVG_LOG_ERROR << "Unknown pair mode";
            exit(EXIT_FAILURE);
        }
    }

    // 3. Save pairs
    OPENMVG_LOG_INFO << "Saving pairs.";
    if (!savePairs(sOutputPairsFilename, pairs)) {
        OPENMVG_LOG_ERROR << "Failed to save pairs to file: \"" << sOutputPairsFilename << "\"";
        exit(EXIT_FAILURE);
    }

    return EXIT_SUCCESS;
}


#endif //SFM_PAIR_GENERATOR_HPP
