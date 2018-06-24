##############################################################################
# CMAKE CONFIGURATION
##############################################################################
cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(iTree3DMap VERSION 1.0.0)
set(CMAKE_BUILD_TYPE Debug)

MESSAGE("\n" "=========================================")
MESSAGE("PROJECT ${PROJECT_NAME} ")
MESSAGE("=========================================")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set corresponding package directories
set(OPENCV_DIR /opt/opencv-3.4.1)
set(PCL_DIR /opt/pcl-1.8.1)

# set corresponding build directories
set(OpenCV_BUILD ${OPENCV_DIR}/build)
set(PCL_BUILD ${PCL_DIR}/build)

# Include dependencies of pcl 1.8.1 in project directorie
set(CMAKE_MODULE_PATH ${PCL_DIR}/cmake/Modules)
message("CMAKE INSTALL PREXFIX = ${CMAKE_INSTALL_PREFIX} (should be /opt/pcl-1.8.1/cmake/Modules)")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

##############################################################################
# PACKAGES
##############################################################################
MESSAGE("Finding packages...")
set(OpenCV_Components core highgui features2d xfeatures2d calib3d imgcodecs imgproc)
find_package(OpenCV 3.4.1 REQUIRED PATHS ${OpenCV_BUILD} COMPONENTS ${OpenCV_Components})
if(OpenCV_FOUND )
    message(STATUS "OpenCV status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: OpenCV 4.0.0 not found")
endif()

#include(include/Eigen/FindEigen.cmake) #Eigen include configuration file for PCL 1.8.1

find_package(Eigen3 3.3.0 REQUIRED)
if(Eigen3_FOUND )
    message(STATUS "Eigen status:")
    message(STATUS "    version: ${Eigen3_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Eigen not found")
endif()

find_package(Boost 1.58.0 REQUIRED)
if(Boost_FOUND )
    message(STATUS "Boost status:")
    message(STATUS "    version: ${Boost_VERSION}")
else()
    message(FATAL_ERROR "    ERROR: Boost not found")
endif()

find_package(Ceres 1.13.0 REQUIRED)
if(Ceres_FOUND )
   message(STATUS "Ceres library status:")
   message(STATUS "    version: ${CERES_VERSION}")
   message(STATUS "    libraries: ${CERES_LIBRARIES}")
   message(STATUS "    include path: ${CERES_INCLUDE_DIRS}")
endif()

set(PCL_Components common io visualization segmentation)
find_package(PCL 1.8.1 REQUIRED PATHS ${PCL_BUILD})
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
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
include(CheckFunctionExists)

# Use the compile definitions defined in PCL
add_definitions(${PCL_DEFINITIONS})

##############################################################################
# LIBRARIES
##############################################################################
LINK_DIRECTORIES(${OpenCV_LIBRARIES})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

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

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES})

MESSAGE("=========================================")
MESSAGE("PROJECT: ${PROJECT_NAME} COMPILED WITH CMAKE " ${CMAKE_VERSION})
MESSAGE("=========================================")
