cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(filter)
# Activate this for use with a debugger
# SET(CMAKE_BUILD_TYPE RelWithDebInfo )
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS}
					)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (filter filter.cpp )
target_link_libraries (filter ${OpenCV_LIBS} ${PCL_LIBRARIES})


#add_executable(pcl_segmentation  pcl_segmentation.cpp)
#target_link_libraries(pcl_segmentation ${OpenCV_LIBS} )
#target_link_libraries(pcl_segmentation ${PCL_LIBRARIES})
