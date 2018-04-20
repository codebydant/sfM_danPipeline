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

cmake_minimum_required(VERSION 3.10.2)
project(Project3DRE_DT VERSION 1.0.0)

set(CMAKE_BUILD_TYPE Debug)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_DIR /home/daniel/Programas/opencv-3.4.1/build)
set(PCL_DIR /home/daniel/Programas/pcl-master/build)
set(EIGEN3_DIR /home/daniel/Programas/eigen-3.3.4/build)

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

find_package(OpenCV 4.0.0 REQUIRED PATHS ${OpenCV_DIR} COMPONENTS core imgproc imgcodecs highgui features2d xfeatures2d)
if(OpenCV_FOUND )
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 4.0.0 not found")
endif()

find_package(Eigen3 3.3.4 REQUIRED PATHS ${EIGEN3_DIR} NO_MODULE)
if(Eigen3_FOUND )
    message(STATUS "Eigen3 library status:")
    message(STATUS "    version: ${Eigen3_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Eigen3 not found")
endif()

find_package(Ceres 1.13.0 REQUIRED)
if(Ceres_FOUND )
    message(STATUS "Ceres library status:")
    message(STATUS "    version: ${CERES_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Ceres not found")
endif()

find_package(Qt5Widgets 5 REQUIRED)
if(Qt5Widgets_FOUND)
    message(STATUS "Qt library status:")
    message(STATUS "    version: ${Qt5Widgets_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Qt not found")
endif()

find_package(PCL 1.8.1 REQUIRED PATHS ${PCL_DIR} COMPONENTS common io segmentation visualization)
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
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

##############################################################################
# SOURCE CODE
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("Finding source files...")
set(SFM_HEADERS "include/Sfm.h" "include/BundleAdjustment.h" "include/Structures.h")
MESSAGE("Found: " ${SFM_HEADERS})
set(SFM_SOURCE "src/Sfm.cpp" "src/BundleAdjustment.cpp")
MESSAGE("Found: " ${SFM_SOURCE})
set(GUI_HEADERS "include/Visualizer.h")
MESSAGE("Found: " ${GUI_HEADERS})
set(GUI_SOURCE "src/Visualizer.cpp")
MESSAGE("Found: " ${GUI_SOURCE})
set(GUI_FILE "src/Visualizer.ui")
MESSAGE("Found: " ${GUI_FILE})
set(MAIN_SOURCE "main.cpp")
MESSAGE("Found: " ${MAIN_SOURCE})
set(CAM_CALIB_SOURCE "src/Camera_calibration.cpp")
MESSAGE("Found: " ${CAM_CALIB_SOURCE})
#FILE(GLOB_RECURSE HEADER_FILES include *.h)
#FILE(GLOB_RECURSE SOURCES_FILES include *.cpp)
#FILE(GLOB_RECURSE GUI_FILE include *.ui)
#add_custom_target(headers SOURCES ${HEADER_FILES} ${SOURCES_FILES} ${GUI_FILE})

##############################################################################
# DATA
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("Generating data example...")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/temple/)
configure_file("data/temple/temple0001.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0002.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0003.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0004.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0005.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0006.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0007.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0008.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0009.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0010.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0011.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0012.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0013.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0014.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0015.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0016.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0017.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0018.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0019.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
configure_file("data/temple/temple0020.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)

configure_file("data/temple/list.txt" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
MESSAGE("Generated: " "build/bin/temple/temple0001.png")
MESSAGE("Generated: " "build/bin/temple/temple0002.png")
MESSAGE("Generated: " "build/bin/temple/temple0003.png")
MESSAGE("Generated: " "build/bin/temple/temple0004.png")
MESSAGE("Generated: " "build/bin/temple/temple0005.png")
MESSAGE("Generated: " "build/bin/temple/temple0006.png")
MESSAGE("Generated: " "build/bin/temple/temple0007.png")
MESSAGE("Generated: " "build/bin/temple/temple0008.png")
MESSAGE("Generated: " "build/bin/temple/temple0009.png")
MESSAGE("Generated: " "build/bin/temple/temple0010.png")
MESSAGE("Generated: " "build/bin/temple/temple0011.png")
MESSAGE("Generated: " "build/bin/temple/temple0012.png")
MESSAGE("Generated: " "build/bin/temple/temple0013.png")
MESSAGE("Generated: " "build/bin/temple/temple0014.png")
MESSAGE("Generated: " "build/bin/temple/temple0015.png")
MESSAGE("Generated: " "build/bin/temple/temple0016.png")
MESSAGE("Generated: " "build/bin/temple/temple0017.png")
MESSAGE("Generated: " "build/bin/temple/temple0018.png")
MESSAGE("Generated: " "build/bin/temple/temple0019.png")
MESSAGE("Generated: " "build/bin/temple/temple0020.png")

##############################################################################
# EXECUTABLES
##############################################################################

set(EXECUTABLE_NAME EM3d)
add_executable(${EXECUTABLE_NAME} ${SFM_HEADERS} ${SFM_SOURCE} ${MAIN_SOURCE}
               ${GUI_HEADERS} ${GUI_SOURCE} ${GUI_FILE} include/PCL_visualizer.h)

##############################################################################
# TARGET LIBRARIES
##############################################################################

set(PCL_LIBRARIES_USED ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES}
                       ${PCL_SEGMENTATION_LIBRARIES})

target_link_libraries(${EXECUTABLE_NAME} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CERES_LIBRARIES} Qt5::Widgets
                      ${PCL_LIBRARIES_USED})

#${catkin_LIBRARIES}

MESSAGE("=========================================")
MESSAGE(" COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")


