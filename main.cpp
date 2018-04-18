/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
#include <QApplication>


/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  //QApplication container(argc, argv);
  //Visualizer window;

  std::ifstream file("temple/list.txt");
  StructFromMotion sf; 
  sf.multithreading(file);




  return 0;

  //return 0;

}//end main
