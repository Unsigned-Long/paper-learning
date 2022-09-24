set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_FILES)

add_library(
        ${CMAKE_PROJECT_NAME}
        STATIC
        ${SRC_FILES}
)

target_include_directories(
        ${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        PUBLIC
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS}
)

add_executable(${CMAKE_PROJECT_NAME}_proj ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_proj
        PRIVATE
        ${CMAKE_PROJECT_NAME}
)

add_executable(${CMAKE_PROJECT_NAME}_viewer ${CMAKE_CURRENT_SOURCE_DIR}/viewer.cpp)

target_include_directories(
        ${CMAKE_PROJECT_NAME}_viewer
        PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_viewer
        PRIVATE
        ${PCL_LIBRARIES}
)
