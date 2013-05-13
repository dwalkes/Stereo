cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( OpenCVReprojectImageToPointCloud )

SET (CMAKE_CXX_COMPILER  "/usr/bin/clang++")
SET (CMAKE_CXX_FLAGS "-Wall -std=c++11")

find_package( PCL 1.2 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(OpenCV_DIR "/opt/opencv244/share/OpenCV/")

find_package( OpenCV REQUIRED )
add_library(DM depthmap.cpp)
target_link_libraries( DM ${OpenCV_LIBS})
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} ${PCL_LIBRARIES} DM)

