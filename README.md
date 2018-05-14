
# A Simple Structure From Motion Pipeline for 3D Reconstruction of a tree

This is a reference implementation of a Structure-from-Motion pipeline in OpenCV, following the work of Roy Shilkrot at SfM-Toy-Library. https://github.com/royshil/SfM-Toy-Library

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this code is to do a 3D reconstruction of a tree. 

----------------------
<img src="https://github.com/danielTobon43/perc_robotic_system3d/tree/master/launch/img.png" align="center" height="200">

Reconstrucción 3D de la geometría de un árbol en nubes de puntos con estimación de características dendrométricas.
* Altura del fuste
* Díametro del fuste
* Volumen de la copa
 

### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

## Compile

To compile use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Prerequisite
- OpenCV 3.4.1: https://github.com/opencv
- ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu


### How to make
1. Download the src code: git@github.com:danielTobon43/perc_robotic_system3d.git and Unpack .zip
2. Rename de package to perc_robotic_system3d without "-master"
3. Copy the package to ROS workspace/src
4.
   Compile with catkin:

	 cd ~/catkin_ws/
	 catkin_make	 

### Test
	 cd ~/catkin_ws
	 source devel/setup.bash
	 roslaunch perc_robotic_system3d perc_robotic_system3d.launch	 

*Note:*
If OpenCV are not install. just compiled. pleas set the path to the current build directory in CMakeList.txt file.



