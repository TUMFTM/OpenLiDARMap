cmake_minimum_required(VERSION 3.20)
project(openlidarmap_cpp VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Iridescence REQUIRED)

find_path(LIBLZF_INCLUDE_DIR lzf.h PATH_SUFFIXES liblzf)
find_library(LIBLZF_LIBRARY NAMES lzf liblzf)

# Fetch external dependencies
include(FetchContent)
FetchContent_Declare(small_gicp GIT_REPOSITORY https://github.com/ga58lar/small_gicp)
FetchContent_MakeAvailable(small_gicp)

# Add library
add_library(
  openlidarmap_lib
  core/registration.cpp
  core/pose_graph.cpp
  core/prediction.cpp
  core/preprocess.cpp
  io/format/kitti.cpp
  io/format/xyz.cpp
  io/format/pcd.cpp
  io/format/ply.cpp
  core/imu_data.cpp
  io/loader_factory.cpp
  io/point_cloud_saver.cpp
  pipeline/openlidarmap.cpp
  utils/file_utils.cpp
  utils/pose_utils.cpp
  utils/helpers.cpp)

target_include_directories(openlidarmap_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} PRIVATE ${small_gicp_INCLUDE_DIRS}
                                                                                       ${LIBLZF_INCLUDE_DIR})

target_link_libraries(openlidarmap_lib PUBLIC Eigen3::Eigen small_gicp Ceres::ceres PRIVATE Iridescence::Iridescence
                                                                                            stdc++fs ${LIBLZF_LIBRARY})

# Add executable
add_executable(openlidarmap_cpp apps/main.cpp)
target_link_libraries(openlidarmap_cpp PRIVATE openlidarmap_lib)
