/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
#include <QApplication>

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  QApplication container(argc, argv);

  StructFromMotion sf;
  sf.interface.show();
  sf.getCameraMatrix("camera-calibration-data.xml");
  sf.imagesLOAD("temple");
  sf.pipeLine();

  return container.exec();

}//end main
