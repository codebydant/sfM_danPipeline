/*********************************
           HEADERS
**********************************/

#include <iostream>
#include <fstream>
#include <string>
#include "../include/Sfm.h"
#include "../include/Ceres_configFile.h"
#include "../include/Bundler_adjustment.h"
#include "../include/Ros_interface.h"

/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/

int main(int argc, char **argv ){

   cv::Mat image1,image2;
   cv::Mat temp_img2;
   std::string frame1,frame2;
   std::ifstream file("temple/list.txt");

std::cout << "************************************************" << std::endl;
std::cout << "************************************************" << std::endl;

   std::getline(file, frame1);
   std::cout << frame1 << std::endl;
   std::getline(file, frame2);
   std::cout << frame2<< std::endl;
   std::cout << "----------------------------------" << std::endl;
   std::cout << "----------------------------------" << std::endl;

  image1 = cv::imread(frame1,0);
  image2 = cv::imread(frame2,0);

  GaussianBlur(image1,image1, cv::Size(7,7),1.5,1.5);
  GaussianBlur(image2,image2, cv::Size(7,7),1.5,1.5);

  StructFromMotion sfm(image1,image2);

  cv::namedWindow( "matches", CV_WINDOW_NORMAL  );
  cv::resizeWindow  ("matches",600,300);
  cv::moveWindow("matches",0,0);
  cv::imshow("matches",sfm.imageMatching());
  cv::waitKey(30);

  std::vector<cv::Point3d> puntos = sfm.triangulation();

  temp_img2 = image2;

  for(int n=1;n<9;n++){

        image1=temp_img2;

        std::getline(file, frame2);
        std::cout << "----------------------------------" << std::endl;
        std::cout << frame2 <<std::endl;
        image2 = cv::imread(frame2,0);
        std::cout << "----------------------------------" << std::endl;

        GaussianBlur(image1,image1, cv::Size(7,7),1.5,1.5);
        GaussianBlur(image2,image2, cv::Size(7,7),1.5,1.5);

        sfm.setConstructor(temp_img2,image2);

        temp_img2 = image2;

        cv::namedWindow( "matches", CV_WINDOW_NORMAL  );
        cv::resizeWindow  ("matches",600,300);
        cv::moveWindow("matches",0,0);
        cv::imshow("matches",sfm.imageMatching());
        cv::waitKey(30);

        std::vector<cv::Point3d> puntos = sfm.triangulation();

        }
  sfm.visualizerPointCloud();


std::cout << "************************************************" << std::endl;
std::cout << "************************************************" << std::endl;

return 0;

}//end main function
