/*********************************
           HEADERS
**********************************/

#include "include/Sfm.h"
//#include "include/Ros_interface.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){


  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  std::ifstream file("temple/list.txt");
  StructFromMotion sf;
  int success = sf.run_SFM(file);

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  return 0;

}//end main
