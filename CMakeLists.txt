##############################################################################
# ROSBUILD
##############################################################################

if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()

##############################################################################
# CMAKE
##############################################################################

cmake_minimum_required(VERSION 3.5.1)
project(proyecto)

##############################################################################
# ROS-INI
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

#rosbuild_include(qt_build qt-ros)

##############################################################################
# PAQUETES
##############################################################################

MESSAGE("*****************************************")
MESSAGE("*****************************************")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND )
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND )
    message(STATUS "Eigen library status:")
    message(STATUS "    version: ${EIGEN_VERSION}")
    message(STATUS "    libraries: ${Eigen_LIBS}")
    message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")
endif()

find_package(Ceres REQUIRED)
if(Ceres_FOUND )
    message(STATUS "Ceres library status:")
    message(STATUS "    version: ${CERES_VERSION}")
    message(STATUS "    libraries: ${CERES_LIBRARIES}")
    message(STATUS "    include path: ${CERES_INCLUDE_DIRS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

##############################################################################
# HEADERS
##############################################################################

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)
include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# LIBRERIAS
##############################################################################

link_directories(${OpenCV_LIB_DIR})
set(SOURCE sfm.cpp)
set(OpenCV_INCLUDE "usr/local/include/opencv2/*.hpp")
FILE(GLOB_RECURSE HEADER_FILES include *.h )
FILE(GLOB_RECURSE HEADERS_OpenCV include /usr/local/include/opencv2/*.hpp )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_custom_target(headers SOURCES ${HEADER_FILES} ${HEADERS_OpenCV})

##############################################################################
# DATA
##############################################################################

configure_file("data/soup1.jpg" ${CMAKE_CURRENT_BINARY_DIR}/bin/soup1.jpg COPYONLY)
configure_file("data/soup2.jpg" ${CMAKE_CURRENT_BINARY_DIR}/bin/soup2.jpg COPYONLY)

##############################################################################
# EJECUTABLES
##############################################################################

add_executable(sfm ${SOURCE} )
#add_executable(cameraCalibration camera_calibration.cpp)
#add_executable(bundler Bundler_Adjustment.cpp)

##############################################################################
# TARGET LIBRARIES
##############################################################################

target_link_libraries(sfm ${OpenCV_LIBS} ${HEADERS_OpenCV} )
target_link_libraries (sfm ${EIGEN3_LIBS} )
target_link_libraries(sfm ${catkin_LIBRARIES})
#target_link_libraries( cameraCalibration ${OpenCV_LIBS})
target_link_libraries(sfm ${CERES_LIBRARIES})
#target_link_libraries (stitching ${EIGEN3_LIBS} )

MESSAGE("*****************************************")
MESSAGE("*****************************************")
























