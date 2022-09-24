
#include "SfM.h"
#include <iostream>

using namespace sfmtoylib;
using namespace std;

int main(int argc, char **argv) {
    const std::string imagesDirectory = "../img/test/";
    const std::string outputPrefix = "../output/";
    constexpr double downscale = 1.0;

    SfM sfm(downscale);
    sfm.setImagesDirectory(imagesDirectory);
    sfm.setConsoleDebugLevel(DebugLogLevel::LOG_INFO);
    sfm.setVisualDebugLevel(DebugLogLevel::LOG_WARN);
    sfm.runSfM();

    //save point cloud and cameras to file
    sfm.saveCloudAndCamerasToPLY(outputPrefix);
}
