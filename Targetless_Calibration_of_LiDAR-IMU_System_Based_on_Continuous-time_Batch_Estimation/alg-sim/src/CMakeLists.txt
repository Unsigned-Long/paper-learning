set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(magic_enum CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/odometer ODOMETER_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/sensor SENSOR_SRC_FILES)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(${CMAKE_PROJECT_NAME} SHARED ${SRC_FILES} ${ODOMETER_SRC_FILES} ${SENSOR_SRC_FILES})

target_include_directories(
        ${CMAKE_PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${SOPHUS_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME} PUBLIC
        magic_enum::magic_enum
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        Sophus::Sophus
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
)

add_executable(${CMAKE_PROJECT_NAME}_proj ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_proj PUBLIC
        ${CMAKE_PROJECT_NAME}
)

