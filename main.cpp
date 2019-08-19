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
               "The program create a 3D reconstruction from an images sequence." << std::endl
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

  /*************************
  STEP X: INPUT IMAGES
  **************************/
  StructFromMotion sf;    
  bool success = sf.imagesLOAD("/home/daniel/Proyecto-grado-3D-recons-master/data/temple");
  if(not success){
      std::cerr << "Error: set of images is not valid." << std::endl;
      return -1;
  }

  /*************************
  STEP X: INPUT CAMERA FILE
  **************************/
  success = sf.getCameraMatrix("/home/daniel/Proyecto-grado-3D-recons-master/data/temple/camera_calibration_template.xml");
  if(not success){
      std::cerr << "Error: camera calibration file is not valid." << std::endl;
      return -1;
  }

  /*************************
  STEP 1: 3D MAPPING
  **************************/
  success = sf.map3D();
  if(not success){
      std::cerr << "Error: Could not obtain 3D Mapping." << std::endl;
      return -1;
  }

  /*************************
  STEP 2: SEGMENTATION
  **************************/
  Segmentation sg;
  sg.color_based_growing_segmentation();

  /*************************
  STEP 3: DENDROMETRY MEASUREMENTS
  **************************/
  Dendrometry tree;
  tree.estimate(sf.cloudPCL);

  return 0;
}
