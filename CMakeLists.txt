cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project( OpenCVReprojectImageToPointCloud )

#SET (CMAKE_CXX_COMPILER  "/usr/bin/clang++")
#SET (CMAKE_CXX_FLAGS "-Wall --std=c++11")

find_package( PCL 1.7 REQUIRED COMPONENTS common visualization segmentation )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# set(OpenCV_DIR "/opt/opencv244/share/OpenCV/")

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_LIBRARY_DIRS})

find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

add_executable( stereoBMtune stereoBMtune.c )
target_link_libraries( stereoBMtune ${OpenCV_LIBS} ${GTK2_LIBRARIES})

add_executable( stereoVartune stereoVartune.cpp )
target_link_libraries( stereoVartune ${OpenCV_LIBS})

add_executable( stereoSGBMtune stereoSGBMtune.cpp )
target_link_libraries( stereoSGBMtune ${OpenCV_LIBS})

add_library(DM depthmap.cpp)
target_link_libraries( DM ${OpenCV_LIBS})

add_executable(RectDM rectDM.cpp)
target_link_libraries( RectDM DM ${OpenCV_LIBS})

add_library(Reprojection reprojection.cpp)
target_link_libraries( Reprojection ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(Segment segment.cpp)
target_link_libraries( Segment ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(Vizualization vizualization.cpp)
target_link_libraries( Vizualization ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( brute brute.cpp )
target_link_libraries( brute ${OpenCV_LIBS})

add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} ${PCL_LIBRARIES} DM Reprojection Segment Vizualization)

