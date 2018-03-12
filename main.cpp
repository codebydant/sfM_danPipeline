/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
//#include "include/Ros_interface.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

  std::ifstream file("temple/list.txt");
  StructFromMotion sf;
  int success = sf.run_SFM(file);

  return 0;

}//end main
