//
// Created by csl on 9/24/22.
//

#ifndef SFM_COMPUTE_MATCHES_HPP
#define SFM_COMPUTE_MATCHES_HPP

#include "openMVG/graph/graph.hpp"
#include "openMVG/graph/graph_stats.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_preemptive_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/stl/stl.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

using namespace openMVG;
using namespace openMVG::matching;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
int computeMatches() {

    std::string sSfM_Data_Filename = Config::outputDir + "/matches/sfm_data.json";
    std::string sOutputMatchesFilename = Config::outputDir + "/matches/matches.putative.bin";
    float fDistRatio = Config::ComputeMatches::distRatio;
    std::string sPredefinedPairList = Config::outputDir + "/matches/pairs.bin";
    std::string sNearestMatchingMethod = Config::ComputeMatches::nearestMatchingMethod;
    bool bForce = Config::ComputeMatches::force;
    unsigned int ui_max_cache_size = 0;

    // Pre-emptive matching parameters
    unsigned int ui_preemptive_feature_count = 200;
    double preemptive_matching_percentage_threshold = 0.08;

    OPENMVG_LOG_INFO << "--input_file " << sSfM_Data_Filename << "\n"
                     << "--output_file " << sOutputMatchesFilename << "\n"
                     << "--pair_list " << sPredefinedPairList << "\n"
                     << "Optional parameters:"
                     << "\n"
                     << "--force " << bForce << "\n"
                     << "--ratio " << fDistRatio << "\n"
                     << "--nearest_matching_method " << sNearestMatchingMethod << "\n"
                     << "--cache_size " << ((ui_max_cache_size == 0) ? "unlimited" : std::to_string(ui_max_cache_size))
                     << "\n"
                     << "--preemptive_feature_used/count " << false << " / " << ui_preemptive_feature_count;

    if (sOutputMatchesFilename.empty()) {
        OPENMVG_LOG_ERROR << "No output file set.";
        return EXIT_FAILURE;
    }

    // -----------------------------
    // . Load SfM_Data Views & intrinsics data
    // . Compute putative descriptor matches
    // + Export some statistics
    // -----------------------------

    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    SfM_Data sfm_data;
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
        OPENMVG_LOG_ERROR << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read.";
        return EXIT_FAILURE;
    }
    const std::string sMatchesDirectory = stlplus::folder_part(sOutputMatchesFilename);

    //---------------------------------------
    // Load SfM Scene regions
    //---------------------------------------
    // Init the regions_type from the image describer file (used for image regions extraction)
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);

    if (!regions_type) {
        OPENMVG_LOG_ERROR << "Invalid: " << sImage_describer << " regions type file.";
        return EXIT_FAILURE;
    }

    //---------------------------------------
    // a. Compute putative descriptor matches
    //    - Descriptor matching (according user method choice)
    //    - Keep correspondences only if NearestNeighbor ratio is ok
    //---------------------------------------

    // Load the corresponding view regions
    std::shared_ptr<Regions_Provider> regions_provider;
    if (ui_max_cache_size == 0) {
        // Default regions provider (load & store all regions in memory)
        regions_provider = std::make_shared<Regions_Provider>();
    } else {
        // Cached regions provider (load & store regions on demand)
        regions_provider = std::make_shared<Regions_Provider_Cache>(ui_max_cache_size);
    }
    // If we use pre-emptive matching, we load fewer regions:
    if (ui_preemptive_feature_count > 0 && false) {
        regions_provider = std::make_shared<Preemptive_Regions_Provider>(ui_preemptive_feature_count);
    }

    // Show the progress on the command line:
    system::LoggerProgress progress;

    if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type, &progress)) {
        OPENMVG_LOG_ERROR << "Cannot load view regions from: " << sMatchesDirectory << ".";
        return EXIT_FAILURE;
    }

    PairWiseMatches map_PutativeMatches;

    // Build some alias from SfM_Data Views data:
    // - List views as a vector of filenames & image sizes
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t>> vec_imagesSize;
    {
        vec_fileNames.reserve(sfm_data.GetViews().size());
        vec_imagesSize.reserve(sfm_data.GetViews().size());
        for (const auto &view_it: sfm_data.GetViews()) {
            const View *v = view_it.second.get();
            vec_fileNames.emplace_back(
                    stlplus::create_filespec(sfm_data.s_root_path, v->s_Img_path)
            );
            vec_imagesSize.emplace_back(v->ui_width, v->ui_height);
        }
    }

    OPENMVG_LOG_INFO << " - PUTATIVE MATCHES - ";
    // If the matches already exists, reload them
    if (!bForce && (stlplus::file_exists(sOutputMatchesFilename))) {
        if (!(Load(map_PutativeMatches, sOutputMatchesFilename))) {
            OPENMVG_LOG_ERROR << "Cannot load input matches file";
            return EXIT_FAILURE;
        }
        OPENMVG_LOG_INFO
        << "\t PREVIOUS RESULTS LOADED;" << " #pair: " << map_PutativeMatches.size();
    } else // Compute the putative matches
    {
        // Allocate the right Matcher according the Matching requested method
        std::unique_ptr<Matcher> collectionMatcher;
        if (sNearestMatchingMethod == "AUTO") {
            if (regions_type->IsScalar()) {
                OPENMVG_LOG_INFO << "Using FAST_CASCADE_HASHING_L2 matcher";
                collectionMatcher = std::make_unique<Cascade_Hashing_Matcher_Regions>(fDistRatio);
            } else if (regions_type->IsBinary()) {
                OPENMVG_LOG_INFO << "Using HNSWHAMMING matcher";
                collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, HNSW_HAMMING);
            }
        } else if (sNearestMatchingMethod == "BRUTEFORCEL2") {
            OPENMVG_LOG_INFO << "Using BRUTE_FORCE_L2 matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, BRUTE_FORCE_L2);
        } else if (sNearestMatchingMethod == "BRUTEFORCEHAMMING") {
            OPENMVG_LOG_INFO << "Using BRUTE_FORCE_HAMMING matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, BRUTE_FORCE_HAMMING);
        } else if (sNearestMatchingMethod == "HNSWL2") {
            OPENMVG_LOG_INFO << "Using HNSWL2 matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, HNSW_L2);
        }
        if (sNearestMatchingMethod == "HNSWL1") {
            OPENMVG_LOG_INFO << "Using HNSWL1 matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, HNSW_L1);
        } else if (sNearestMatchingMethod == "HNSWHAMMING") {
            OPENMVG_LOG_INFO << "Using HNSWHAMMING matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, HNSW_HAMMING);
        } else if (sNearestMatchingMethod == "ANNL2") {
            OPENMVG_LOG_INFO << "Using ANN_L2 matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, ANN_L2);
        } else if (sNearestMatchingMethod == "CASCADEHASHINGL2") {
            OPENMVG_LOG_INFO << "Using CASCADE_HASHING_L2 matcher";
            collectionMatcher = std::make_unique<Matcher_Regions>(fDistRatio, CASCADE_HASHING_L2);
        } else if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2") {
            OPENMVG_LOG_INFO << "Using FAST_CASCADE_HASHING_L2 matcher";
            collectionMatcher = std::make_unique<Cascade_Hashing_Matcher_Regions>(fDistRatio);
        }
        if (!collectionMatcher) {
            OPENMVG_LOG_ERROR << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod;
            return EXIT_FAILURE;
        }
        // Perform the matching
        system::Timer timer;
        {
            // From matching mode compute the pair list that have to be matched:
            Pair_Set pairs;
            if (sPredefinedPairList.empty()) {
                OPENMVG_LOG_INFO << "No input pair file set. Use exhaustive match by default.";
                const size_t NImage = sfm_data.GetViews().size();
                pairs = exhaustivePairs(NImage);
            } else if (!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs)) {
                OPENMVG_LOG_ERROR << "Failed to load pairs from file: \"" << sPredefinedPairList << "\"";
                return EXIT_FAILURE;
            }
            OPENMVG_LOG_INFO << "Running matching on #pairs: " << pairs.size();
            // Photometric matching of putative pairs
            collectionMatcher->Match(regions_provider, pairs, map_PutativeMatches, &progress);

            //---------------------------------------
            //-- Export putative matches & pairs
            //---------------------------------------
            if (!Save(map_PutativeMatches, std::string(sOutputMatchesFilename))) {
                OPENMVG_LOG_ERROR << "Cannot save computed matches in: " << sOutputMatchesFilename;
                return EXIT_FAILURE;
            }
            // Save pairs
            const std::string sOutputPairFilename =
                    stlplus::create_filespec(sMatchesDirectory, "preemptive_pairs", "txt");
            if (!savePairs(sOutputPairFilename, getPairs(map_PutativeMatches))) {
                OPENMVG_LOG_ERROR << "Cannot save computed matches pairs in: " << sOutputPairFilename;
                return EXIT_FAILURE;
            }
        }
        OPENMVG_LOG_INFO << "Task (Regions Matching) done in (s): " << timer.elapsed();
    }

    OPENMVG_LOG_INFO << "#Putative pairs: " << map_PutativeMatches.size();

    // -- export Putative View Graph statistics
    graph::getGraphStatistics(sfm_data.GetViews().size(), getPairs(map_PutativeMatches));

    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(
            vec_fileNames.size(),
            map_PutativeMatches,
            stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg")
    );
    //-- export view pair graph once putative graph matches has been computed
    {
        std::set<IndexT> set_ViewIds;
        std::transform(
                sfm_data.GetViews().begin(),
                sfm_data.GetViews().end(),
                std::inserter(set_ViewIds, set_ViewIds.begin()),
                stl::RetrieveKey()
        );

        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativeMatches));
        graph::exportToGraphvizData(
                stlplus::create_filespec(sMatchesDirectory, "putative_matches"), putativeGraph
        );
    }

    return EXIT_SUCCESS;
}


#endif //SFM_COMPUTE_MATCHES_HPP
