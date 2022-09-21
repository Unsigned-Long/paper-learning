set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(GTest)

add_executable(${CMAKE_PROJECT_NAME}_test ${CMAKE_CURRENT_SOURCE_DIR}/testmain.cpp)

target_include_directories(
        ${CMAKE_PROJECT_NAME}_test PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${GTEST_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/src/include
)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}_test PRIVATE
        ${CMAKE_PROJECT_NAME}
        ${GTEST_BOTH_LIBRARIES}
        pthread
)