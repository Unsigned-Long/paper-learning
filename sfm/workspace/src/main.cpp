#include "artwork/logger/logger.h"

#include "ComputeFeatures.hpp"
#include "ComputeMatches.hpp"
#include "config.h"
#include "fMInit_ImageListing.hpp"
#include "GeometricFilter.hpp"
#include "global_SfM.hpp"
#include "PairGenerator.hpp"
#include "ComputeSfM_DataColor.hpp"

int main(int argc, char **argv) {
    OPENMVG_LOG_INFO << "\n[fMInitImageListing] start\n";
    if (fMInitImageListing() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[fMInitImageListing] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[fMInitImageListing] finished\n";

    OPENMVG_LOG_INFO <<"\n[computeFeatures] start\n";
    if (computeFeatures() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeFeatures] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[computeFeatures] finished\n";

    OPENMVG_LOG_INFO <<"\n[pairGenerator] start\n";
    if (pairGenerator() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[pairGenerator] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[pairGenerator] finished\n";

    OPENMVG_LOG_INFO <<"\n[computeMatches] start\n";
    if (computeMatches() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeMatches] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[computeMatches] finished\n";

    OPENMVG_LOG_INFO <<"\n[geometricFilter] start\n";
    if (geometricFilter() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[geometricFilter] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[geometricFilter] finished\n";

    OPENMVG_LOG_INFO <<"\n[globalSfM] start\n";
    if (globalSfM() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[globalSfM] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[globalSfM] finished\n";

    OPENMVG_LOG_INFO <<"\n[computeSfMDataColor] start\n";
    if (computeSfMDataColor() == EXIT_FAILURE) {
        OPENMVG_LOG_ERROR << "[computeSfMDataColor] failed.";
    }
    OPENMVG_LOG_INFO <<"\n[computeSfMDataColor] finished\n";

    return EXIT_SUCCESS;
}