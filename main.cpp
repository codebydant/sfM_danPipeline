/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
#include <QApplication>
#include "include/Visualizer.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  std::ifstream file("temple/list.txt");
  StructFromMotion sf;
 // int success = sf.run_SFM(file);

  QApplication container(argc, argv);
  Visualizer window;
  window.show();

  return container.exec();

}//end main
