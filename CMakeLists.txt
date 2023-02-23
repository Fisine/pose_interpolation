project(pose_interpolation)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 14)



find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pose_interpolation pose_interpolation.cpp)

target_link_libraries(pose_interpolation Eigen3::Eigen ${OpenCV_LIBS})