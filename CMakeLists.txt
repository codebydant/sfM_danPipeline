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

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)



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
MESSAGE("****** BUSCANDO LIBRERIAS ************")
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
    message(STATUS "    version: ${Eigen3_VERSION}")
    message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")
endif()

find_package(Ceres REQUIRED)
if(Ceres_FOUND )
    message(STATUS "Ceres library status:")
    message(STATUS "    version: ${CERES_VERSION}")
    message(STATUS "    include path: ${CERES_INCLUDE_DIR}")
endif()

find_package(Qt5Widgets)
if(Qt5Widgets_FOUND)
    message(STATUS "Qt5 library status:")
    message(STATUS "    version: ${Qt5Widgets_VERSION}")
    message(STATUS "    libraries: ${Qt5Widgets_LIBRARIES}")
    message(STATUS "    include path: ${Qt5Widgets_INCLUDE_DIRS}")
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
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)
include_directories(${catkin_INCLUDE_DIRS})

# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})


##############################################################################
# LIBRERIAS
##############################################################################

link_directories(${OpenCV_LIB_DIR})
link_directories(${Qt5Widgets_LIBRARIES})

##############################################################################
# VARIABLES
##############################################################################
set(SFM_HEADERS "include/Sfm.h" "include/BundleAdjustment.h")
set(QT_HEADERS "include/Visualizer.h" )
set(SOURCES_SFM "src/Sfm.cpp" "src/BundleAdjustment.cpp")
set(SOURCE_MAIN "main.cpp" )
set(SOURCE_QT "src/Visualizer.cpp")
set(SRC_CALIB "src/Camera_calibration.cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
FILE(GLOB_RECURSE HEADER_FILES include *.h )
FILE(GLOB_RECURSE SOURCES_FILES include *.cpp )
FILE(GLOB_RECURSE GUI_FILE include *.ui )
add_custom_target(headers SOURCES ${HEADER_FILES} ${SOURCES_FILES} ${GUI_FILE})

##############################################################################
# DATA
##############################################################################

configure_file("data/temple/temple0001.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)

##############################################################################
# EJECUTABLES
##############################################################################

add_executable(sfm ${SOURCE_MAIN} ${SOURCES_SFM} ${SFM_HEADERS} ${SOURCE_QT} ${QT_HEADERS}  )

#add_executable(calibration ${SRC_CALIB})



##############################################################################
# TARGET LIBRARIES
##############################################################################

target_link_libraries(sfm ${OpenCV_LIBS} ${EIGEN3_LIBS} ${CERES_LIBRARIES} Qt5::Widgets)
#target_link_libraries(calibration ${OpenCV_LIBS})
 #${catkin_LIBRARIES}

MESSAGE("*****************************************")
MESSAGE("*****************************************")
























