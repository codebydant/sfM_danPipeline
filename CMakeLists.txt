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
project(PercRoboticSystem3D VERSION 1.0.0)

#set(CMAKE_BUILD_TYPE Debug)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_DIR /home/daniel/Programas/opencv-3.4.1/build)
set(PCL_DIR /home/daniel/Programas/pcl-master/build)
set(Ceres_DIR /home/daniel/Programas/ceres-1.14.0/build)

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

find_package(OpenCV 4.0.0 REQUIRED PATHS ${OpenCV_DIR} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND)
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 4.0.0 not found")
endif()

find_package(Ceres 1.14.0 REQUIRED PATHS ${Ceres_DIR} )
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

set(PCL_Components common io visualization)

find_package(PCL 1.8.1 REQUIRED PATHS ${PCL_DIR} COMPONENTS ${PCL_Components})
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
#############################################################################
#set("${VTK_LEGACY_SILENT} ON")
#set("${VTK_LEGACY_REMOVE} ON")

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})


# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${Qt5Widgets_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
LINK_DIRECTORIES(${CERES_LIBRARIES})


##############################################################################
# SOURCE CODE
##############################################################################

set(SFM_HEADERS "include/Sfm.h" "include/BundleAdjustment.h")
set(SFM_SOURCE "src/Sfm.cpp" "src/BundleAdjustment.cpp")
set(GUI_HEADERS "include/Visualizer.h")
set(GUI_SOURCE "src/Visualizer.cpp")
set(GUI_FILE "src/Visualizer.ui")
set(MAIN_SOURCE "main.cpp")
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
configure_file("data/camera-calibration-data.xml" ${CMAKE_CURRENT_BINARY_DIR}/bin COPYONLY)
MESSAGE("Generated data:/build/bin/temple")

##############################################################################
# EXECUTABLES
##############################################################################

add_executable(${PROJECT_NAME} ${SFM_HEADERS} ${SFM_SOURCE} ${MAIN_SOURCE}
               ${GUI_HEADERS} ${GUI_SOURCE} ${GUI_FILE} include/Utilities.h)

##############################################################################
# TARGET LIBRARIES
##############################################################################

set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
                     opencv_imgproc opencv_imgcodecs)       

set(PCL_LIBS ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES}
                      ${Qt5Widgets_LIBRARIES} ${PCL_LIBS} )



#${catkin_LIBRARIES}

MESSAGE("=========================================")
MESSAGE(" COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")


