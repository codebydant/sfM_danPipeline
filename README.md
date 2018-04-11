
# Proyecto-grado-3D-recons
----------------------
<img src="https://projects.asl.ethz.ch/datasets/lib/exe/fetch.php?cache=&w=900&h=539&tok=dd850d&media=laserregistration:gazebo_winter:tree.png" align="center" height="200">

Reconstrucción 3D de la geometría de un árbol en nubes de puntos con estimación de características dasométricas.
* Altura del fuste
* Díametro del fuste
* Volumen de la copa
 
   
### Resources

* Homepage: <http://opencv.org>
* Docs: <http://docs.opencv.org/master/>

### Requerimientos

* OpenCV 3.2.0
* OpenCV contrib 3.2.0
* QT Creator 5.9.1
* CMake 3.10.2
* Eigen3 3.3.4
* Ceres 1.13.0
* ROS Kinetic
* VTK 8.1.0

### Building
Download the src code: git@github.com:danielTobon43/Proyecto-grado-3D-recons.git

Unpack .zip

mkdir build && cd build

cmake ../

make

### Test
cd /_build_directory/bin

./EM3d

### Nota:
El fichero CMakeLists.txt sirve para compilar el programa con CMake 3.10.2.



