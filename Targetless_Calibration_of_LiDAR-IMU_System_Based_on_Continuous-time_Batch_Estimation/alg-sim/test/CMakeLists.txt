set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(GTest)

add_executable(${CMAKE_PROJECT_NAME}_test_main ${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp)

target_include_directories(
        ${CMAKE_PROJECT_NAME}_test_main PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${GTEST_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src/include
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_test_main PRIVATE
        ${CMAKE_PROJECT_NAME}
        ${GTEST_BOTH_LIBRARIES}
        pthread
)

add_executable(${CMAKE_PROJECT_NAME}_test_lidar_odometer ${CMAKE_CURRENT_SOURCE_DIR}/test_lidar_odometer.cpp)

target_include_directories(
        ${CMAKE_PROJECT_NAME}_test_lidar_odometer PUBLIC
        ${CMAKE_SOURCE_DIR}/src/include
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_test_lidar_odometer PRIVATE
        ${CMAKE_PROJECT_NAME}
        pthread
)