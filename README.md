
# Proyecto-grado-3D-recons
----------------------
<img src="https://projects.asl.ethz.ch/datasets/lib/exe/fetch.php?cache=&w=900&h=539&tok=dd850d&media=laserregistration:gazebo_winter:tree.png" align="center" height="200">

Reconstrucción 3D de la geometría de un árbol en nubes de puntos con estimación de características dasométricas.
* Altura del fuste
* Díametro del fuste
* Volumen de la copa
  
  yeah
  
### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

### Requerimientos

* OpenCV 3.2.0
* OpenCV contrib 3.2.0
* QT Creator 4.8
* PCL Library 
* Cmake 10.2
* Eigen
* Ceres
* ROS Kinetic

### Building
Download the src code: git@github.com:danielTobon43/Proyecto-grado-3D-recons.git

Unpack 3D_recons.zip

mkdir build && cd build

cmake ../src/

make

### Test
cd /3D_recons_build_directory/bin

./sfm

### Nota:
El fichero CMakeLists.txt sirve para compilar el programa con CMake.
El fichero CMakeLists.txt.user es utilizado por QTCreator para compilar el programa.


