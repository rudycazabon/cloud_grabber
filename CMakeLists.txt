cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt/5.5/gcc_64)

project(cloud_grabber)

find_package(PCL 1.7 REQUIRED)
find_package (Qt5 REQUIRED COMPONENTS Widgets Core) 


include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${PCL_INCLUDE_DIRS} ~/librealsense/include )
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib /usr/lib/x86_64-linux-gnu)
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_grabber cloud_grabber.cpp)
target_link_libraries (cloud_grabber ${PCL_LIBRARIES} /usr/local/lib/librealsense.so)
