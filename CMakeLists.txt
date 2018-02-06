cmake_minimum_required(VERSION 3.5.1)
project(Proyecto_Grado)
MESSAGE("*****************************************")
MESSAGE("*****************************************")
find_package(OpenCV REQUIRED COMPONENTS core imgproc viz xfeatures2d features2d)
if(OpenCV_FOUND )
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND )
 MESSAGE("Eigen ----> Status: Found")
endif()
Set( EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}" )
find_package(catkin REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(roscpp REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include(CheckFunctionExists)
include_directories(${catkin_INCLUDE_DIRS})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
link_directories(${OpenCV_LIB_DIR})
set(SOURCE sfm.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

configure_file("data/soup1.jpg" ${CMAKE_CURRENT_BINARY_DIR}/bin/soup1.jpg COPYONLY)
configure_file("data/soup2.jpg" ${CMAKE_CURRENT_BINARY_DIR}/bin/soup2.jpg COPYONLY)
add_executable(sfm ${SOURCE})
add_executable(cameraCalibration camera_calibration.cpp)

target_link_libraries(sfm ${OpenCV_LIBS} )
target_link_libraries (sfm ${EIGEN3_LIBS} )
target_link_libraries(sfm ${catkin_LIBRARIES})
target_link_libraries( cameraCalibration ${OpenCV_LIBS})
#target_link_libraries (stitching ${EIGEN3_LIBS} )

MESSAGE("*****************************************")
MESSAGE("*****************************************")
























