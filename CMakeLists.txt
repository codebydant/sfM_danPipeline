##############################################################################
# ROS CONFIGURATION
##############################################################################

#if(ROSBUILD)
 # include(rosbuild.cmake OPTIONAL)
  #return()
#endif()

##############################################################################
# CMAKE CONFIGURATION
##############################################################################

cmake_minimum_required(VERSION 3.5.1)
project(PercRoboticSystem3D VERSION 1.0.0)

set(CMAKE_BUILD_TYPE Debug)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_DIR /opt/opencv-3.4.1/build)
set(PCL_DIR /opt/pcl-1.8.1/build)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# ROS-INI
##############################################################################

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

#rosbuild_include(qt_build qt-ros)

##############################################################################
# PACKAGES
##############################################################################

MESSAGE("Finding packages...")

set(OpenCV_Components core highgui features2d calib3d)
find_package(OpenCV 3.4.0 REQUIRED PATHS ${OpenCV_DIR} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND )
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 4.0.0 not found")
endif()

include(include/Eigen/FindEigen.cmake) #Eigen include configuration file for PCL 1.8.1

set(PCL_Components common io visualization)
find_package(PCL 1.8.0 REQUIRED PATHS ${PCL_DIR} COMPONENTS ${PCL_Components})
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: PCL not found")
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

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

##############################################################################
# SOURCE CODE
##############################################################################

set(UTILITIE_HEADER "include/Utilities.h")
set(SFM_HEADER "include/Sfm.h")
set(SFM_SOURCE "src/Sfm.cpp")
set(VISUALIZER_HEADER "include/Visualizer.h")
set(VISUALIZER_SOURCE "src/Visualizer.cpp")
set(BUNDLE_HEADER "include/BundleAdjustment.h")
set(BUNDLE_SOURCE "src/BundleAdjustment.cpp")
set(MAIN_SOURCE "main.cpp")

##############################################################################
# DATA
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("Generating data example...")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/temple/)
configure_file("data/temple/temple0001.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0002.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0003.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0004.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0005.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0006.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0007.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0008.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0009.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0010.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0011.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0012.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0013.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0014.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0015.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0016.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0017.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0018.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0019.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/temple/temple0020.png" ${EXECUTABLE_OUTPUT_PATH}/temple COPYONLY)
configure_file("data/camera-calibration-data.xml" ${EXECUTABLE_OUTPUT_PATH} COPYONLY)
MESSAGE("Generated data:/build/bin/temple")
MESSAGE("Generated data:/build/bin/camera-calibration-data.xml")

##############################################################################
# EXECUTABLES
##############################################################################

add_executable(${PROJECT_NAME} ${SFM_HEADER} ${SFM_SOURCE} ${VISUALIZER_HEADER}
               ${VISUALIZER_SOURCE} ${BUNDLE_HEADER} ${BUNDLE_SOURCE} ${UTILITIE_HEADER} ${MAIN_SOURCE})

##############################################################################
# TARGET LIBRARIES
##############################################################################

set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
                     opencv_imgproc opencv_imgcodecs)

set(PCL_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBS})

#${catkin_LIBRARIES}

MESSAGE("=========================================")
MESSAGE(" COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")



