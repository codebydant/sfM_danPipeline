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
  bool success = sf.imagesLOAD("/home/daniel/Proyecto-grado-3D-recons-master/data/temple");
  if(not success){
      std::cerr << "Error: set of images is not valid." << std::endl;
      return -1;
  }
  success = sf.getCameraMatrix("/home/daniel/Proyecto-grado-3D-recons-master/data/temple/camera_calibration_template.xml");
  if(not success){
      std::cerr << "Error: camera calibration file is not valid." << std::endl;
      return -1;
  }

  success = sf.run_SFM();
  if(not success){
      std::cerr << "Error: Could not obtain 3D Mapping." << std::endl;
      return -1;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZRGB> ());

  for(size_t i = 0; i < sf.nReconstructionCloud.size(); ++i){
      Point3D pt3d =  sf.nReconstructionCloud[i];
      cv::Vec3b rgbv(255,255,255);
      pcl::PointXYZRGB pclp;
      pclp.x  = pt3d.pt.x;
      pclp.y  = pt3d.pt.y;
      pclp.z  = pt3d.pt.z;
      rgbv =  sf.nReconstructionCloudRGB[i];

      // RGB color, needs to be represented as an integer
      uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
      pclp.rgb = *reinterpret_cast<float*>(&rgb);
      cloudPCL->push_back(pclp);
   }

   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->header.frame_id ="map3d";
   cloudPCL->is_dense = false;


  Segmentation sg;
  sg.color_based_growing_segmentation(cloudPCL);

  return 0;

}
