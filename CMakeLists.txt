##############################################################################
# ROSBUILD
##############################################################################

#if(ROSBUILD)
 # include(rosbuild.cmake OPTIONAL)
  #return()
#endif()

##############################################################################
# CMAKE
##############################################################################

cmake_minimum_required(VERSION 3.5.1)
project(proyecto)
set(CMAKE_BUILD_TYPE Debug)

##############################################################################
# ROS-INI
##############################################################################

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
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

#find_package(catkin REQUIRED COMPONENTS
 # pcl_conversions
 # pcl_ros
 # roscpp
 # sensor_msgs
#)



##############################################################################
# HEADERS
##############################################################################
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)
include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# LIBRERIAS
##############################################################################

link_directories(${OpenCV_LIB_DIR})

##############################################################################
# VARIABLES
##############################################################################
set(SFM_HEADER "include/Sfm.h")
set(SOURCE "main.cpp" "src/Sfm.cpp" "src/Structures.cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
FILE(GLOB_RECURSE HEADER_FILES include *.h )
FILE(GLOB_RECURSE SOURCES_FILES include *.cpp )
#FILE(GLOB_RECURSE HEADERS_OpenCV include /usr/local/include/opencv2/*.hpp )
add_custom_target(headers SOURCES ${HEADER_FILES} ${SOURCES_FILES})

##############################################################################
# DATA
##############################################################################

configure_file("data/temple" ${CMAKE_CURRENT_BINARY_DIR}/bin/ COPYONLY)
configure_file("data/camera-calibration-data.xml" ${CMAKE_CURRENT_BINARY_DIR}/bin/camera-calibration-data.xml COPYONLY)

##############################################################################
# EJECUTABLES
##############################################################################

add_executable(sfm ${SOURCE}  ${SFM_HEADER} )
#add_executable(calibCamera Camera_calibration.cpp)

##############################################################################
# TARGET LIBRARIES
##############################################################################

target_link_libraries(sfm ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CERES_LIBRARIES})
#target_link_libraries(calibCamera ${OpenCV_LIBS})
 #${catkin_LIBRARIES}


MESSAGE("*****************************************")
MESSAGE("*****************************************")
























