/*********************************
           HEADERS
**********************************/
#include "include/Sfm.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/
// This function displays the help
void help(char** arg1){
  std::cout << "----------------------------------------------- \n"
               "The program create a 3D reconstruction from a images sequence." << std::endl
            << "Usage:\n" << arg1[0] << " <image sequence> " << " <camera calibration file.xml>" << std::endl
            << "'esc' --> for quit the program" <<
            "\n -----------------------------------------------"<< std::endl;
}

int main(int argc, char **argv){

  /**
  // COMMAND LINE INPUT
  cv::CommandLineParser parser(argc, argv, "{help h||}{@input1||}{@input2||}");
  if(parser.has("help")){
      std::cout << "----------------------------------------------- \n"
                   "The program create a 3D reconstruction from a images sequence." << std::endl
                << "Usage:\n" << "<image sequence> " << "<camera calibration file.xml>" << std::endl
                << "'esc' --> for quit the program" <<
                "\n -----------------------------------------------"<< std::endl;
      return 1;
   }

  /* IMAGES PATH INPUT
   std::string arg1 = parser.get<std::string>("@input1");
   if(arg1.empty()){
       help(argv);
       return 1;
   }

   /* CAMERA CALIBRATION INPUT
   std::string arg2 = parser.get<std::string>("@input2");
   if(arg2.empty()){
       help(argv);
       return 1;
   }*/

  StructFromMotion sf;    
  sf.imagesLOAD("temple");
  sf.getCameraMatrix("camera-calibration-data.xml");
  sf.run_SFM();

  return 0;

}//end main
