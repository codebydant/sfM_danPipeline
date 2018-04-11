/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
#include <QApplication>


/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  std::ifstream file("temple/list.txt");
  std::cout << "yeah"<<std::endl;
  StructFromMotion sf;
  int success = sf.run_SFM(file);
  /*
  QApplication container(argc, argv);
  Visualizer window; 
  //window.show(); 



  //return container.exec();
  */
  return 0;

}//end main
