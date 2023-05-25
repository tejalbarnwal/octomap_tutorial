cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(octomap_tutorial)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})


add_executable (createOctomap createOctomap.cpp)
target_link_libraries(createOctomap octomap)
