cmake_minimum_required(VERSION 3.15)
project(kalman_filter)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(kf app/main.cpp ${SOURCE_FILES})
target_link_libraries(kf yaml-cpp)