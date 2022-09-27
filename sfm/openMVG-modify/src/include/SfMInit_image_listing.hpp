//
// Created by csl on 9/24/22.
//

#ifndef SFM_SFMINIT_IMAGE_LISTING_HPP
#define SFM_SFMINIT_IMAGE_LISTING_HPP

#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/system/loggerprogress.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "config.h"

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;

/// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
/// With f,ppx,ppy as valid numerical value
bool checkIntrinsicStringValidity(const std::string &Kmatrix, double &focal, double &ppx, double &ppy) {
    std::vector<std::string> vec_str;
    stl::split(Kmatrix, ';', vec_str);
    if (vec_str.size() != 9) {
        OPENMVG_LOG_ERROR << "\n Missing ';' character";
        return false;
    }
    // Check that all K matrix value are valid numbers
    for (size_t i = 0; i < vec_str.size(); ++i) {
        double readValue = 0.0;
        std::stringstream ss;
        ss.str(vec_str[i]);
        if (!(ss >> readValue)) {
            OPENMVG_LOG_ERROR << "\n Used an invalid not a number character";
            return false;
        }
        if (i == 0) focal = readValue;
        if (i == 2) ppx = readValue;
        if (i == 5) ppy = readValue;
    }
    return true;
}

bool getGPS(const std::string &filename,
            const int &GPS_to_XYZ_method,
            Vec3 &pose_center) {
    std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
    if (exifReader) {
        // Try to parse EXIF metadata & check existence of EXIF data
        if (exifReader->open(filename) && exifReader->doesHaveExifInfo()) {
            // Check existence of GPS coordinates
            double latitude, longitude, altitude;
            if (exifReader->GPSLatitude(&latitude) &&
                exifReader->GPSLongitude(&longitude) &&
                exifReader->GPSAltitude(&altitude)) {
                // Add ECEF or UTM XYZ position to the GPS position array
                switch (GPS_to_XYZ_method) {
                    case 1:
                        pose_center = lla_to_utm(latitude, longitude, altitude);
                        break;
                    case 0:
                    default:
                        pose_center = lla_to_ecef(latitude, longitude, altitude);
                        break;
                }
                return true;
            }
        }
    }
    return false;
}


/// Check string of prior weights
std::pair<bool, Vec3> checkPriorWeightsString(const std::string &sWeights) {
    std::pair<bool, Vec3> val(true, Vec3::Zero());
    std::vector<std::string> vec_str;
    stl::split(sWeights, ';', vec_str);
    if (vec_str.size() != 3) {
        OPENMVG_LOG_ERROR << "Missing ';' character in prior weights";
        val.first = false;
    }
    // Check that all weight values are valid numbers
    for (size_t i = 0; i < vec_str.size(); ++i) {
        double readValue = 0.0;
        std::stringstream ss;
        ss.str(vec_str[i]);
        if (!(ss >> readValue)) {
            OPENMVG_LOG_ERROR << "Used an invalid not a number character in local frame origin";
            val.first = false;
        }
        val.second[i] = readValue;
    }
    return val;
}

//
// Create the description of an input image dataset for OpenMVG toolsuite
// - Export a SfM_Data file with View & Intrinsic data
//
int fMInitImageListing() {

    std::string sImageDir = Config::imgDir;
    std::string sfileDatabase = Config::fileDatabasePath;
    std::string sOutputDir = Config::outputDir + "/matches";
    // "f;0;ppx;0;f;ppy;0;0;1"
    std::string sKmatrix = Config::SfMInitImageListing::KMatrix;

    std::string sPriorWeights = Config::SfMInitImageListing::priorWeights;
    std::pair<bool, Vec3> prior_w_info(false, Vec3());

    int i_User_camera_model = Config::SfMInitImageListing::cameraModel;

    bool b_Group_camera_model = Config::SfMInitImageListing::groupCameraModel;

    int i_GPS_XYZ_method = Config::SfMInitImageListing::GPS_XYZ_Method;

    double focal_pixels = Config::SfMInitImageListing::focalPixels;

    const bool b_Use_pose_prior = Config::SfMInitImageListing::usePosePrior;
    OPENMVG_LOG_INFO << "\n--imageDirectory " << sImageDir
                     << "\n--sensorWidthDatabase " << sfileDatabase
                     << "\n--outputDirectory " << sOutputDir
                     << "\n--focal " << focal_pixels
                     << "\n--intrinsics " << sKmatrix
                     << "\n--camera_model " << i_User_camera_model
                     << "\n--group_camera_model " << b_Group_camera_model
                     << "\n--use_pose_prior " << b_Use_pose_prior
                     << "\n--prior_weights " << sPriorWeights
                     << "\n--gps_to_xyz_method " << i_GPS_XYZ_method;

    // Expected properties for each image
    double width = -1, height = -1, focal = -1, ppx = -1, ppy = -1;

    const auto e_User_camera_model = EINTRINSIC(i_User_camera_model);

    if (!stlplus::folder_exists(sImageDir)) {
        OPENMVG_LOG_ERROR << "The input directory doesn't exist";
        return EXIT_FAILURE;
    }

    if (sOutputDir.empty()) {
        OPENMVG_LOG_ERROR << "Invalid output directory";
        return EXIT_FAILURE;
    }

    if (!stlplus::folder_exists(sOutputDir)) {
        if (!stlplus::folder_create(sOutputDir)) {
            OPENMVG_LOG_ERROR << "Cannot create output directory";
            return EXIT_FAILURE;
        }
    }

    if (sKmatrix.size() > 0 && !checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy)) {
        OPENMVG_LOG_ERROR << "Invalid K matrix input";
        return EXIT_FAILURE;
    }

    if (sKmatrix.size() > 0 && focal_pixels != -1.0) {
        OPENMVG_LOG_ERROR << "Cannot combine -f and -k options";
        return EXIT_FAILURE;
    }

    std::vector<Datasheet> vec_database;
    if (!sfileDatabase.empty()) {
        if (!parseDatabase(sfileDatabase, vec_database)) {
            OPENMVG_LOG_ERROR << "Invalid input database: " << sfileDatabase
                              << ", please specify a valid file.";
            return EXIT_FAILURE;
        }
    }

    // Check if prior weights are given
    if (b_Use_pose_prior) {
        prior_w_info = checkPriorWeightsString(sPriorWeights);
    }

    std::vector<std::string> vec_image = stlplus::folder_files(sImageDir);
    std::sort(vec_image.begin(), vec_image.end());

    // Configure an empty scene with Views and their corresponding cameras
    SfM_Data sfm_data;
    sfm_data.s_root_path = sImageDir; // Setup main image root_path
    Views &views = sfm_data.views;
    Intrinsics &intrinsics = sfm_data.intrinsics;

    system::LoggerProgress my_progress_bar(vec_image.size(), "- Listing images -");
    std::ostringstream error_report_stream;
    for (auto iter_image = vec_image.begin(); iter_image != vec_image.end(); ++iter_image, ++my_progress_bar) {
        // Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
        width = height = ppx = ppy = focal = -1.0;

        const std::string sImageFilename = stlplus::create_filespec(sImageDir, *iter_image);
        const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

        // Test if the image format is supported:
        if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown) {
            error_report_stream << sImFilenamePart << ": Unknown image file format." << "\n";
            continue; // image cannot be opened
        }

        if (sImFilenamePart.find("mask.png") != std::string::npos
            || sImFilenamePart.find("_mask.png") != std::string::npos) {
            error_report_stream
                    << sImFilenamePart << " is a mask image" << "\n";
            continue;
        }

        ImageHeader imgHeader{};
        if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
            continue; // image cannot be read

        width = imgHeader.width;
        height = imgHeader.height;
        ppx = width / 2.0;
        ppy = height / 2.0;


        // Consider the case where the focal is provided manually
        if (sKmatrix.size() > 0) // Known user calibration K matrix
        {
            if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
                focal = -1.0;
        } else // User provided focal length value
        if (focal_pixels != -1)
            focal = focal_pixels;

        // If not manually provided or wrongly provided
        if (focal == -1) {
            std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
            exifReader->open(sImageFilename);

            const bool bHaveValidExifMetadata = exifReader->doesHaveExifInfo()
                                                && !exifReader->getModel().empty()
                                                && !exifReader->getBrand().empty();

            if (bHaveValidExifMetadata) // If image contains meta data
            {
                // Handle case where focal length is equal to 0
                if (exifReader->getFocal() == 0.0f) {
                    error_report_stream
                            << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
                    focal = -1.0;
                } else
                    // Create the image entry in the list file
                {
                    const std::string sCamModel = exifReader->getBrand() + " " + exifReader->getModel();

                    Datasheet datasheet;
                    if (getInfo(sCamModel, vec_database, datasheet)) {
                        // The camera model was found in the database, so we can compute it's approximated focal length
                        const double ccdw = datasheet.sensorSize_;
                        focal = std::max(width, height) * exifReader->getFocal() / ccdw;
                    } else {
                        error_report_stream
                                << stlplus::basename_part(sImageFilename)
                                << "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
                                << "Please consider add your camera model and sensor width in the database." << "\n";
                    }
                }
            }
        }
        // Build intrinsic parameter related to the view
        std::shared_ptr<IntrinsicBase> intrinsic;

        if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0) {
            // Create the desired camera type
            switch (e_User_camera_model) {
                case PINHOLE_CAMERA:
                    intrinsic = std::make_shared<Pinhole_Intrinsic>
                            (width, height, focal, ppx, ppy);
                    break;
                case PINHOLE_CAMERA_RADIAL1:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
                            (width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_RADIAL3:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_BROWN:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case PINHOLE_CAMERA_FISHEYE:
                    intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
                            (width, height, focal, ppx, ppy, 0.0, 0.0, 0.0,
                             0.0); // setup no distortion as initial guess
                    break;
                case CAMERA_SPHERICAL:
                    intrinsic = std::make_shared<Intrinsic_Spherical>
                            (width, height);
                    break;
                default:
                    OPENMVG_LOG_ERROR << "Error: unknown camera model: " << (int) e_User_camera_model;
                    return EXIT_FAILURE;
            }
        }

        // Build the view corresponding to the image
        Vec3 pose_center;
        if (getGPS(sImageFilename, i_GPS_XYZ_method, pose_center) && b_Use_pose_prior) {
            ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

            // Add intrinsic related to the image (if any)
            if (!intrinsic) {
                //Since the view have invalid intrinsic data
                // (export the view, with an invalid intrinsic field value)
                v.id_intrinsic = UndefinedIndexT;
            } else {
                // Add the defined intrinsic to the sfm_container
                intrinsics[v.id_intrinsic] = intrinsic;
            }

            v.b_use_pose_center_ = true;
            v.pose_center_ = pose_center;
            // prior weights
            if (prior_w_info.first == true) {
                v.center_weight_ = prior_w_info.second;
            }

            // Add the view to the sfm_container
            views[v.id_view] = std::make_shared<ViewPriors>(v);
        } else {
            View v(*iter_image, views.size(), views.size(), views.size(), width, height);

            // Add intrinsic related to the image (if any)
            if (!intrinsic) {
                //Since the view have invalid intrinsic data
                // (export the view, with an invalid intrinsic field value)
                v.id_intrinsic = UndefinedIndexT;
            } else {
                // Add the defined intrinsic to the sfm_container
                intrinsics[v.id_intrinsic] = intrinsic;
            }

            // Add the view to the sfm_container
            views[v.id_view] = std::make_shared<View>(v);
        }
    }

    // Display saved warning & error messages if any.
    if (!error_report_stream.str().empty()) {
        OPENMVG_LOG_WARNING << "Warning & Error messages:\n" << error_report_stream.str();
    }

    // Group camera that share common properties if desired (leads to faster & stable BA).
    if (b_Group_camera_model) {
        GroupSharedIntrinsics(sfm_data);
    }

    // Store SfM_Data views & intrinsic data
    if (!Save(sfm_data, stlplus::create_filespec(sOutputDir, "sfm_data.json"), ESfM_Data(VIEWS | INTRINSICS))) {
        return EXIT_FAILURE;
    }

    OPENMVG_LOG_INFO << "SfMInit_ImageListing report:\n"
                     << "listed #File(s): " << vec_image.size() << "\n"
                     << "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
                     << "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size();

    return EXIT_SUCCESS;
}


#endif //SFM_SFMINIT_IMAGE_LISTING_HPP
