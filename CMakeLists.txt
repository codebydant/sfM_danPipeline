##############################################################################
# CMAKE CONFIGURATION
##############################################################################

cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(perc_robotic_system3d VERSION 1.0.0)

set(CMAKE_BUILD_TYPE Debug)

MESSAGE("\n" "=========================================")
MESSAGE(" COMPILING PROJECT  ")
MESSAGE("=========================================")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(OpenCV_DIR /opt/opencv-3.4.1/build)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# PACKAGES
##############################################################################

MESSAGE("Finding packages...")
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pcl_ros pcl_conversions sensor_msgs)
if(catkin_FOUND )
    message(STATUS "catkin status:")
    message(STATUS "    version: ${catkin_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: catkin not found")
endif()

set(OpenCV_Components core highgui features2d calib3d opencv_imgproc opencv_imgcodecs)
find_package(OpenCV 3.4.0 REQUIRED PATHS ${OpenCV_DIR} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND )
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV ${OpenCV_VERSION} not found")
endif()

##############################################################################
# HEADERS
##############################################################################

catkin_metapackage()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES proyecto_grado
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp std_msgs
#  DEPENDS system_lib
)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES
##############################################################################

LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

##############################################################################
# SOURCE CODE
##############################################################################

set(UTILITIE_HEADER "include/Utilities.h")
set(SFM_HEADER "include/Sfm.h")
set(SFM_SOURCE "src/Sfm.cpp")
set(ROS_INTERFACE_HEADER "include/Ros_interface.h")
set(ROS_INTERFACE_SOURCE "src/Ros_interface.cpp")
set(BUNDLE_HEADER "include/BundleAdjustment.h")
set(BUNDLE_SOURCE "src/BundleAdjustment.cpp")
set(SEGMENT_HEADER "include/Segmentation.h")
set(SEGMENT_SOURCE "src/Segmentation.cpp")
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################

add_executable(${PROJECT_NAME} ${SFM_HEADER} ${SFM_SOURCE} ${ROS_INTERFACE_HEADER} ${SEGMENT_HEADER}
               ${SEGMENT_SOURCE} ${ROS_INTERFACE_SOURCE} ${BUNDLE_HEADER} ${BUNDLE_SOURCE} ${UTILITIE_HEADER}
               ${MAIN_SOURCE})

##############################################################################
# TARGET LIBRARIES
##############################################################################

set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
                     opencv_imgproc opencv_imgcodecs)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})

MESSAGE("=========================================")
MESSAGE(" COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")

