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

cmake_minimum_required(VERSION 3.10.2)
set(BUILD_SHARED_LIBS ON)
project(Project3DRE_DT VERSION 1.0.0)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILANDO PROYECTO  ")
MESSAGE("=========================================")

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_INCLUDE_DIR /usr/local/include/opencv)
set(OpenCV_LIBS_DIR /usr/local/lib)

##############################################################################
# ROS-INI
##############################################################################

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()

#rosbuild_include(qt_build qt-ros)

##############################################################################
# PAQUETES
##############################################################################

MESSAGE("BUSCANDO DEPENDENCIAS...")

find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV)
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

find_package(Qt5Widgets REQUIRED)
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

MESSAGE("[DONE]")
MESSAGE("---------------------------------------")
MESSAGE("INCLUYENDO DIRECTORIOS...")

##############################################################################
# HEADERS
##############################################################################
include_directories( ${OpenCV_INCLUDE_DIR} )
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)
include_directories(${catkin_INCLUDE_DIRS})

MESSAGE("[DONE]")
# Use the compile definitions defined in the Qt 5 Widgets module
add_definitions(${Qt5Widgets_DEFINITIONS})

##############################################################################
# LIBRERIAS
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("VINCULANDO LIBRERÍAS...")

LINK_DIRECTORIES(${OpenCV_LIBS_DIR})
link_directories(${Qt5Widgets_LIBRARIES})

MESSAGE("[DONE]")

##############################################################################
# VARIABLES
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("VINCULANDO ARCHIVOS FUENTE...")

set(SFM_HEADERS "include/Sfm.h" "include/BundleAdjustment.h")
set(QT_HEADERS "include/Visualizer.h")
set(SOURCES_SFM "src/Sfm.cpp" "src/BundleAdjustment.cpp")
set(SOURCE_MAIN "main.cpp")
set(SOURCE_QT "src/Visualizer.cpp")
set(SRC_CALIB "src/Camera_calibration.cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE SOURCES_FILES include *.cpp)
FILE(GLOB_RECURSE GUI_FILE include *.ui)
#add_custom_target(headers SOURCES ${HEADER_FILES} ${SOURCES_FILES} ${GUI_FILE})

MESSAGE("[DONE]")

##############################################################################
# DATA
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("GENERANDO DATOS DE EJEMPLOS...")

configure_file("data/temple/temple0001.png" ${CMAKE_CURRENT_BINARY_DIR}/bin/temple COPYONLY)

MESSAGE("[DONE]")

##############################################################################
# EJECUTABLES
##############################################################################

MESSAGE("---------------------------------------")
MESSAGE("CREANDO EJECUTABLE...")

add_executable(sfm ${SOURCE_MAIN} ${SOURCES_SFM} ${SFM_HEADERS} ${SOURCE_QT} ${QT_HEADERS} src/Visualizer.ui)

MESSAGE("[DONE]")



##############################################################################
# TARGET LIBRARIES
##############################################################################
set(OpenCVV_LIBS libopencv_core.so libopencv_features2d.so libopencv_imgproc.so libopencv_viz.so libopencv_highgui.so libopencv_imgcodecs.so libopencv_xfeatures2d.so
    libopencv_calib3d.so)

target_link_libraries(sfm ${OpenCVV_LIBS} ${EIGEN3_LIBS} ${CERES_LIBRARIES} Qt5::Widgets)
#target_link_libraries(calibration ${OpenCV_LIBS})
 #${catkin_LIBRARIES}

 MESSAGE("=========================================")
 MESSAGE(" COMPILACIÓN TERMINADA")
 MESSAGE("=========================================")

