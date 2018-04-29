
# A Simple Structure From Motion Pipeline for 3D Reconstruction of a tree

This is a reference implementation of a Structure-from-Motion pipeline in OpenCV, following the work of Roy Shilkrot at SfM-Toy-Library. https://github.com/royshil/SfM-Toy-Library

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this code is to do a 3D recostruction of a tree. 

----------------------
<img src="https://projects.asl.ethz.ch/datasets/lib/exe/fetch.php?cache=&w=900&h=539&tok=dd850d&media=laserregistration:gazebo_winter:tree.png" align="center" height="200">

Reconstrucción 3D de la geometría de un árbol en nubes de puntos con estimación de características dasométricas.
* Altura del fuste
* Díametro del fuste
* Volumen de la copa
 
   
### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

## Compile

To compile use CMake minimum required 3.10.2 : https://github.com/Kitware/CMake

### Prerequisite
- OpenCV 3.4.1: https://github.com/opencv
- Ceres Solver (for bundle adjustment): https://github.com/ceres-solver/ceres-solver
- Boost C++ libraries v1.54+: http://www.boost.org/
- Eigen3: https://github.com/RLovelett/eigen
- PCL 1.8.1: https://github.com/PointCloudLibrary/pcl
- VTK 8.1.0: https://github.com/Kitware/VTK
- QT Creator 5.9.1: https://www.qt.io/download-qt-installer?hsCtaTracking=9f6a2170-a938-42df-a8e2-a9f0b1d6cdce%7C6cb0de4f-9bb5-4778-ab02-bfb62735f3e5

### How to make
Download the src code: git@github.com:danielTobon43/Proyecto-grado-3D-recons.git and Unpack .zip

	 mkdir build && cd build
	 cmake ../
	 make

### Test
	 cd /_build_directory/bin
	 ./EM3d

*Note:*
El fichero CMakeLists.txt sirve para compilar el programa con CMake 3.10.2.



