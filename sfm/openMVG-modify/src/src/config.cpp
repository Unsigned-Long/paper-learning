//
// Created by csl on 9/24/22.
//
#include "config.h"
#include "yaml-cpp/yaml.h"

std::string Config::imgDir = {};
std::string Config::outputDir = {};
std::string Config::fileDatabasePath = {};

std::string Config::SfMInitImageListing::priorWeights = {};
int Config::SfMInitImageListing::cameraModel = {};
int Config::SfMInitImageListing::GPS_XYZ_Method = {};
bool Config::SfMInitImageListing::usePosePrior = {};
double Config::SfMInitImageListing::focalPixels = {};
bool Config::SfMInitImageListing::groupCameraModel = {};

bool Config::ComputeFeatures::upRight = {};
bool Config::ComputeFeatures::force = {};
std::string Config::ComputeFeatures::imageDescriberMethod = {};

std::string Config::PairGenerator::pairMode = {};
int Config::PairGenerator::contiguousCount = {};

float Config::ComputeMatches::distRatio = {};
std::string Config::ComputeMatches::nearestMatchingMethod = {};
bool Config::ComputeMatches::force = {};

std::string Config::GeometricFilter::geometricModel = {};
bool Config::GeometricFilter::force = {};
bool Config::GeometricFilter::guidedMatching = {};
int Config::GeometricFilter::maxIteration = {};

std::string Config::GlobalSfM::engineName = {};

void Config::loadConfig(const std::string &configFilePath) {
    auto doc = YAML::LoadFile(configFilePath);

    Config::imgDir = doc["imgDir"].as<std::string>();
    Config::outputDir = doc["outputDir"].as<std::string>();
    Config::fileDatabasePath = doc["fileDatabasePath"].as<std::string>();

    auto SfMInitImageListing = doc["SfMInitImageListing"];
    SfMInitImageListing::priorWeights = SfMInitImageListing["priorWeights"].as<std::string>();
    SfMInitImageListing::cameraModel = SfMInitImageListing["cameraModel"].as<int>();
    SfMInitImageListing::focalPixels = SfMInitImageListing["focalPixels"].as<double>();
    SfMInitImageListing::GPS_XYZ_Method = SfMInitImageListing["GPS_XYZ_Method"].as<int>();
    SfMInitImageListing::usePosePrior = SfMInitImageListing["usePosePrior"].as<bool>();
    SfMInitImageListing::groupCameraModel = SfMInitImageListing["groupCameraModel"].as<bool>();

    auto ComputeFeatures = doc["ComputeFeatures"];
    ComputeFeatures::upRight = ComputeFeatures["upRight"].as<bool>();
    ComputeFeatures::force = ComputeFeatures["force"].as<bool>();
    ComputeFeatures::imageDescriberMethod = ComputeFeatures["imageDescriberMethod"].as<std::string>();

    auto PairGenerator = doc["PairGenerator"];
    PairGenerator::pairMode = PairGenerator["pairMode"].as<std::string>();
    PairGenerator::contiguousCount = PairGenerator["contiguousCount"].as<int>();

    auto ComputeMatches = doc["ComputeMatches"];
    ComputeMatches::distRatio = ComputeMatches["distRatio"].as<float>();
    ComputeMatches::nearestMatchingMethod = ComputeMatches["nearestMatchingMethod"].as<std::string>();
    ComputeMatches::force = ComputeMatches["force"].as<bool>();

    auto GeometricFilter = doc["GeometricFilter"];
    GeometricFilter::geometricModel = GeometricFilter["geometricModel"].as<std::string>();
    GeometricFilter::force = GeometricFilter["force"].as<bool>();
    GeometricFilter::guidedMatching = GeometricFilter["guidedMatching"].as<bool>();
    GeometricFilter::maxIteration = GeometricFilter["maxIteration"].as<int>();

    auto GlobalSfM = doc["GlobalSfM"];
    GlobalSfM::engineName = GlobalSfM["engineName"].as<std::string>();

}
