#include "artwork/logger/logger.h"

#include "compute_features.hpp"
#include "compute_matches.hpp"
#include "config.h"
#include "fMInit_image_listing.hpp"
#include "geometric_filter.hpp"
#include "global_SfM.hpp"
#include "pair_generator.hpp"
#include "compute_SfM_data_color.hpp"

int main(int argc, char **argv) {
    if (fMInitImageListing() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[fMInitImageListing] failed.";
    }

    if (computeFeatures() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeFeatures] failed.";
    }

    if (pairGenerator() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[pairGenerator] failed.";
    }

    if (computeMatches() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeMatches] failed.";
    }

    if (geometricFilter() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[geometricFilter] failed.";
    }

    std::unique_ptr<ReconstructionEngine> sfm_engine;
    if (globalSfM(sfm_engine) == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[globalSfM] failed.";
    }

    if (computeSfMDataColor() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeSfMDataColor] failed.";
    }

    auto sfmData = sfm_engine->Get_SfM_Data();

    // methods to get SfM info
    sfmData.GetPoses();
    sfmData.GetControl_Points();
    sfmData.GetLandmarks();

    return EXIT_SUCCESS;
}