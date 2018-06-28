##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

# set project name
project(iTree3DMap VERSION 1.0.0)

# set build type = Debug mode
set(CMAKE_BUILD_TYPE Release)

message("\n" "=========================================")
message("Project: ${PROJECT_NAME} ")
message("=========================================")

# set the include directive in the same project folder
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set corresponding package directories
set(OpenCV_DIR /opt/opencv-3.4.1/build)
set(PCL_DIR /opt/pcl-1.8.1/build)

# set the CMP0074 policy to old behavior (disable warnings)
cmake_policy(SET CMP0074 OLD)

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/../cmake/Modules)

# set cmake for use std c++11 and output executable folder to bin
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# set turn off the output rule messages of cmake
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

##############################################################################
# PACKAGES
##############################################################################
message("Finding packages...")

message("***********************")
message("OpenCV PACKAGE")
message("***********************")

set(OpenCV_Components core highgui features2d xfeatures2d calib3d imgcodecs imgproc)
find_package(OpenCV 3.4.1 REQUIRED PATHS ${OPENCV_DIR} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND)
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV ${OpenCV_VERSION} not found")
endif()

message("***********************")
message("Ceres PACKAGE")
message("***********************")

find_package(Ceres 1.13.0 REQUIRED)
if(Ceres_FOUND)
   message(STATUS "Ceres library status:")
   message(STATUS "    version: ${CERES_VERSION}")
else()
   message(FATAL_ERROR "    ERROR: Ceres ${CERES_VERSION} not found")
endif()

message("***********************")
message("PCL PACKAGE")
message("***********************")

find_package(PCL 1.8.1 REQUIRED PATHS ${PCL_DIR})
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: PCL not found")
endif()

##############################################################################
# HEADERS
##############################################################################
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
#add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES PATH
##############################################################################
link_directories(${OpenCV_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CERES_LIBRARIES})

##############################################################################
# SOURCE CODE
##############################################################################
set(UTILITIE_HEADER "include/Utilities.h")
set(SFM_HEADER "include/Sfm.h")
set(SFM_SOURCE "src/Sfm.cpp")
set(BUNDLE_HEADER "include/BundleAdjustment.h")
set(BUNDLE_SOURCE "src/BundleAdjustment.cpp")
set(SEGMENT_HEADER "include/Segmentation.h")
set(SEGMENT_SOURCE "src/Segmentation.cpp")
set(DENDROMETRY_HEADER "include/DendrometryE.h")
set(DENDROMETRY_SOURCE "src/DendrometryE.cpp")
set(MAIN_SOURCE "main.cpp")

##############################################################################
# EXECUTABLES
##############################################################################
add_executable(${PROJECT_NAME} ${SFM_HEADER} ${SFM_SOURCE} ${SEGMENT_HEADER} ${SEGMENT_SOURCE}
               ${BUNDLE_HEADER} ${BUNDLE_SOURCE} ${UTILITIE_HEADER} ${DENDROMETRY_HEADER} ${DENDROMETRY_SOURCE}
               ${MAIN_SOURCE})

##############################################################################
# TARGET LIBRARIES
##############################################################################
set(OpenCV_LIBRARIES opencv_core opencv_highgui opencv_features2d opencv_calib3d
                     opencv_imgproc opencv_imgcodecs opencv_xfeatures2d)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

message("=========================================")
message("Project: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
message("=========================================")
