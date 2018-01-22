
cmake_minimum_required(VERSION 3.5.1)
project(Proyecto_Grado)
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
#find_package(Qt4 REQUIRED)
#find_package(catkin REQUIRED COMPONENTS cv_bridge pcl pcl_ros)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(3D_recons main.cpp)
add_executable(cameraCalibration camera_calibration.cpp)
add_executable(stitching Stitching.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(EXECUTABLE_OUTPUT_PATH "bin")
Set( EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}" )
target_link_libraries( 3D_recons ${OpenCV_LIBS})
target_link_libraries( cameraCalibration ${OpenCV_LIBS})
target_link_libraries( stitching ${OpenCV_LIBS})
target_link_libraries (3D_recons ${EIGEN3_LIBS} )
#target_link_libraries (stitching ${EIGEN3_LIBS} )
#target_link_libraries(3D_recons ${catkin_LIBRARIES})
target_link_libraries(3D_recons ${PCL_LIBRARIES})
#target_link_libraries(3D_recons Qt4::QtGui)
#target_link_libraries(stitching Qt4::QtGui)
#target_link_libraries(stitching Qt4::QtCore)






















