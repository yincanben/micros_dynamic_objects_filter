cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(moving_object_filter)
# Activate this for use with a debugger
# SET(CMAKE_BUILD_TYPE RelWithDebInfo )
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS}
					)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (moving_object_filter src/main.cpp src/moving_object_filter.cpp)
target_link_libraries (moving_object_filter ${OpenCV_LIBS} ${PCL_LIBRARIES})
