cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_merge)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER "g++")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_BUILD_TYPE debug)

#pcl opencv boost
find_package(PCL 1.7 REQUIRED)
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED )
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options filesystem REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( ${OPENCV_INCLUDE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (color_pointcloud  color_pointcloud.cc)
target_link_libraries (color_pointcloud
			${PCL_LIBRARIES} 
			${OpenCV_LIBS})

add_executable (kitti_premerge  kitti_premerge.cc)
target_link_libraries (kitti_premerge
			${PCL_LIBRARIES} 
			${OpenCV_LIBS})

add_executable (kitti_merge  kitti_merge.cc)
target_link_libraries (kitti_merge
			${PCL_LIBRARIES} 
			${OpenCV_LIBS})