set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Ceres REQUIRED)
find_package(OpenMVG REQUIRED)

add_definitions(-DOPENMVG_USE_OPENMP)

add_executable(${PROJECT_NAME}_proj ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_include_directories(
        ${PROJECT_NAME}_proj
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        "/usr/local/include/openMVG_dependencies/"
)

target_link_libraries(
        ${PROJECT_NAME}_proj
        PRIVATE
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_features
        OpenMVG::openMVG_exif
        libvlsift.a
        OpenMVG::openMVG_matching_image_collection
)