/*********************************
           HEADERS
**********************************/

#include <iostream>
#include <fstream>
#include <string>
#include "../include/Sfm.h"
#include "../include/Bundler_adjustment.h"
#include "../include/Ros_interface.h"
//#include "Sfm.cpp"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

   cv::Mat image1,image2,temp_img2;
   std::string frame1,frame2;
   std::ifstream file("temple/list.txt");

std::cout << "************************************************" << std::endl;
std::cout << "************************************************" << std::endl;

  std::cout << "----------------------------------" << std::endl;
  std::getline(file, frame1);
  std::cout << frame1 << std::endl;
  std::getline(file, frame2);
  std::cout << frame2<< std::endl;
  std::cout << "----------------------------------" << std::endl;

  image1 = cv::imread(frame1,CV_LOAD_IMAGE_COLOR);
  image2 = cv::imread(frame2,CV_LOAD_IMAGE_COLOR);

   StructFromMotion sf;


  std::vector<cv::Point3d> cloud =sf.recon(image1,image2);

  //sfm.initTriangulation();
  temp_img2 = image2;
/*
  for(int n=1;n<3;n++){

        image1=temp_img2;
        std::cout << "----------------------------------" << std::endl;
        std::getline(file, frame2);        
        std::cout << frame2 <<std::endl;
        image2 = cv::imread(frame2,CV_LOAD_IMAGE_COLOR);
        std::cout << "----------------------------------" << std::endl;

        sfm.setConstructor(temp_img2,image2);
        temp_img2 = image2;
        sfm.matchingImShow();
        sfm.initTriangulation();

        }

*/

/*
              for (int i = 0; i < bal_problem.num_observations(); ++i) {
                ceres::CostFunction* cost_function =
                    SnavelyReprojectionError::Create(
                         bal_problem.observations()[2 * i + 0],
                         bal_problem.observations()[2 * i + 1]);
                problem.AddResidualBlock(cost_function,
                                         NULL ,
                                         bal_problem.mutable_camera_for_observation(i),
                                         bal_problem.mutable_point_for_observation(i));
              }
*/


std::cout << "************************************************" << std::endl;
std::cout << "************************************************" << std::endl;

return 0;

}//end main function
