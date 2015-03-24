cmake_minimum_required(VERSION 2.8) 
project(warpImage)  
find_package(OpenCV REQUIRED)  
add_executable(warpImage warpImage.cpp)  
target_link_libraries(warpImage ${OpenCV_LIBS})  

