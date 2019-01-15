cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(underground_filter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (underground_filter underground_filter.cpp)
target_link_libraries (underground_filter ${PCL_LIBRARIES})