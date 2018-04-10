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

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_INCLUDE_DIR /usr/local/include/opencv)
set(OpenCV_INCLUDE_DIR2 /usr/local/include/opencv2)
set(OpenCV_LIBS_DIR /usr/local/lib)

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

MESSAGE("FINDING DEPENDS...")

find_package(OpenCV 3 REQUIRED PATHS /home/daniel/Programas/opencv-3.2.0)
if(OpenCV_FOUND )
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 3.2.0 not found")
endif()

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND )
    message(STATUS "Eigen library status:")
    message(STATUS "    version: ${Eigen3_VERSION}")
    message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "    ERROR: Eigen not found")
endif()

find_package(Ceres REQUIRED)
if(Ceres_FOUND )
    message(STATUS "Ceres library status:")
    message(STATUS "    version: ${CERES_VERSION}")
    message(STATUS "    include path: ${CERES_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "    ERROR: Ceres not found")
endif()

find_package(Qt5Widgets REQUIRED)
if(Qt5Widgets_FOUND)
    message(STATUS "Qt5 library status:")
    message(STATUS "    version: ${Qt5Widgets_VERSION}")
    message(STATUS "    libraries: ${Qt5Widgets_LIBRARIES}")
    message(STATUS "    include path: ${Qt5Widgets_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "    ERROR: Qt5 not found")
endif()

#find_package(catkin REQUIRED COMPONENTS
 # pcl_conversions
 # pcl_ros
 # roscpp
 # sensor_msgs
#)

MESSAGE("[DONE]")

##############################################################################
# HEADERS
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("INCLUDING DIRECTORIES...")
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR} )
MESSAGE("Include: " ${OpenCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIR2} )
MESSAGE("Include: " ${OpenCV_INCLUDE_DIR2})
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
MESSAGE("Include: " ${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
MESSAGE("Include: " ${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
MESSAGE("Include: " ${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
MESSAGE("Include: " ${catkin_INCLUDE_DIRS})
MESSAGE("[DONE]")
# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("LINKING LIBRARIES...")
LINK_DIRECTORIES(${OpenCV_LIBS_DIR})
MESSAGE("Link: " ${OpenCV_LIBS_DIR})
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES})
MESSAGE("Link: " ${Qt5Widgets_LIBRARIES})
MESSAGE("[DONE]")

##############################################################################
# SOURCE CODE
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("FINDING SOURCE CODE...")
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
MESSAGE("[DONE]")

##############################################################################
# DATA
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("GENERATING DATA EXAMPLES...")
configure_file("data/temple/temple0001.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)
MESSAGE("Generated: " "data/temple/temple0001.png")
MESSAGE("[DONE]")

##############################################################################
# EXECUTABLES
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("CREATING EXECUTABLE FILE...")
add_executable(sfm ${MAIN_SOURCE} include/prueba.h ${GUI_HEADERS} ${GUI_SOURCE} ${GUI_FILE})
MESSAGE("Created: " sfm)
MESSAGE("[DONE]")

##############################################################################
# TARGET LIBRARIES
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("TARGET LIBRARIES...")
set(OpenCVV_LIBS libopencv_core.so libopencv_features2d.so libopencv_imgproc.so libopencv_viz.so
                 libopencv_highgui.so libopencv_imgcodecs.so libopencv_xfeatures2d.so libopencv_calib3d.so)

target_link_libraries(sfm ${OpenCVV_LIBS} ${EIGEN3_LIBS} ${CERES_LIBRARIES} Qt5::Widgets)
MESSAGE("Target libraries: " ${OpenCVV_LIBS})
MESSAGE("Target libraries: " Eigen3)
MESSAGE("Target libraries: " ${CERES_LIBRARIES})
#target_link_libraries(calibration ${OpenCV_LIBS})
#${catkin_LIBRARIES}

MESSAGE("=========================================")
MESSAGE(" COMPILED")
MESSAGE("=========================================")

