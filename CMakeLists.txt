cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2bin_converter)

find_package(PCL 1.8 REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(stdc++fs)
link_libraries(-lstdc++fs)

add_executable (pcd2bin_converter convert.cpp)

target_link_libraries (pcd2bin_converter ${PCL_LIBRARIES})
