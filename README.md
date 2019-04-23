
# A simple structure from motion pipeline for 3d incremental reconstruction

This is a reference implementation of a Structure-from-Motion pipeline in OpenCV, following the work of Roy Shilkrot at SfM-Toy-Library. https://github.com/royshil/SfM-Toy-Library

*Note:* This is not a complete and robust SfM pipeline implementation. The purpose of this code is to do a 3D reconstruction of a tree and get dendrometry estimation. 

----------------------

A simple incremental SFM pipeline for 3D reconstruction of a tree with bundle adjustment. 
* Incremental SFM
* Bundle Adjustment - Ceres solver
* Densify cloud - PMVS2 

### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

## Build 

To build use CMake minimum required 3.5.1 : https://github.com/Kitware/CMake

### Prerequisite
- OpenCV 3.4.1: https://github.com/opencv
- PCL 1.8.1

### How to make
* Download the src code: git@github.com:danielTobon43/perc_robotic_system3d.git and Unpack .zip
Compile with cmake


	cmake ../
	make
 	 
### Test
	cd ~/buil_directory
	./iTree3DMap

*Note:*
If OpenCV are not install. just compiled. pleas set the path to the current build directory in CMakeList.txt file.



