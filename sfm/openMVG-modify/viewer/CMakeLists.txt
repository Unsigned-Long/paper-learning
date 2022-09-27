set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

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
