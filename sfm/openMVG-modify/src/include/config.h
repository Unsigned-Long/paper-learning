//
// Created by csl on 9/24/22.
//

#ifndef SFM_CONFIG_H
#define SFM_CONFIG_H

#include "string"

struct Config {
    static std::string outputDir;
    static std::string imgDir;
    static std::string fileDatabasePath;

    struct SfMInitImageListing {
        static std::string priorWeights;
        static int cameraModel;
        static int GPS_XYZ_Method;
        static bool usePosePrior;
        static double focalPixels;
        static bool groupCameraModel;
    };
    struct ComputeFeatures {
        static bool upRight;
        static bool force;
        static std::string imageDescriberMethod;
    };
    struct PairGenerator {
        static std::string pairMode;
        static int contiguousCount;
    };
    struct ComputeMatches {
        static float distRatio;
        static std::string nearestMatchingMethod;
        static bool force;
    };
    struct GeometricFilter {
        static std::string geometricModel;
        static bool force;
        static bool guidedMatching;
        static int maxIteration;
    };
    struct GlobalSfM {
        static std::string engineName;
    };

    static void loadConfig(const std::string &configFilePath);
};

#endif //SFM_CONFIG_H
