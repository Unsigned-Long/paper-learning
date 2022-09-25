set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Ceres REQUIRED)
find_package(OpenMVG REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions(-DOPENMVG_USE_OPENMP)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(${CMAKE_PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(
        ${CMAKE_PROJECT_NAME}
        PUBLIC
        ${YAML_CPP_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        "/usr/local/include/openMVG_dependencies/"
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        PUBLIC
        OpenMVG::openMVG_sfm
        OpenMVG::openMVG_matching
        OpenMVG::openMVG_features
        OpenMVG::openMVG_exif
        OpenMVG::openMVG_matching_image_collection
        libvlsift.a
        ${YAML_CPP_LIBRARIES}
)

add_executable(${CMAKE_PROJECT_NAME}_proj ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}_proj
        PRIVATE
        ${CMAKE_PROJECT_NAME}
)