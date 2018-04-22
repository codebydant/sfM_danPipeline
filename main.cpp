/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
#include <QApplication>

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  StructFromMotion sf;   
  sf.getCameraMatrix("camera-calibration-data.xml");
  sf.imagesLOAD("temple");  
  sf.run_SFM();

  return 0;

}//end main
