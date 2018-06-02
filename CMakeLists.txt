##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
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

find_package(Eigen3 3.3.0 REQUIRED)
if(Eigen3_FOUND )
    message(STATUS "Eigen status:")
    message(STATUS "    version: ${Eigen3_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Eigen not found")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
if(Boost_FOUND )
    message(STATUS "Boost status:")
    message(STATUS "    version: ${Boost_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Boost not found")
endif()

set(PCL_Components common io visualization segmentation)
find_package(PCL 1.8.0 REQUIRED PATHS ${PCL_DIR} COMPONENTS ${PCL_Components})
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: PCL not found")
endif()

##############################################################################
# HEADERS
##############################################################################
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(/home/daniel/Downloads/sba-master)
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES
##############################################################################
LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
LINK_DIRECTORIES(/home/daniel/Downloads/sba-master/build)

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
set(SEGMENT_HEADER "include/Segmentation.h")
set(SEGMENT_SOURCE "src/Segmentation.cpp")
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${SFM_HEADER} ${SFM_SOURCE} ${VISUALIZER_HEADER} ${SEGMENT_HEADER} ${SEGMENT_SOURCE}
               ${VISUALIZER_SOURCE} ${BUNDLE_HEADER} ${BUNDLE_SOURCE} ${UTILITIE_HEADER} ${MAIN_SOURCE})

##############################################################################
# TARGET LIBRARIES
##############################################################################
set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
                     opencv_imgproc opencv_imgcodecs)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} sba)

MESSAGE("=========================================")
MESSAGE(" COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")
