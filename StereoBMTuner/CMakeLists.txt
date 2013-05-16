cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( OpenCVReprojectImageToPointCloud )

SET (CMAKE_CXX_COMPILER  "/usr/bin/clang++")
SET (CMAKE_CXX_FLAGS "-Wall -std=c++11")

set(OpenCV_DIR "/opt/opencv244/share/OpenCV/")

find_package( OpenCV REQUIRED )
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARY_DIRS})

add_executable( stereoBMtune stereoBMtune.c )
target_link_libraries( stereoBMtune ${OpenCV_LIBS} ${GTK2_LIBRARIES})

add_executable( stereoVartune stereoVartune.cpp )
target_link_libraries( stereoVartune ${OpenCV_LIBS})
