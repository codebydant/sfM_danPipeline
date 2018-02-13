/*********************************
           HEADERS
**********************************/


#include <iostream>
#include <fstream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <string>
#include <mutex>
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
   std::ifstream file("temple/list.txt");
   std::string frame1,frame2;


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
            // sfm.visualizerPointCloud();

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


/*********************************
        FUNCIONES-CUERPO
**********************************/


cv::Mat triangulate(const cv::Mat& p1,const cv::Mat& p2, cv::Vec2d& u1, cv::Vec2d& u2){
// system of equations assuming image=[u,v] and X=[x,y,z,1]
// from u(p3.X)= p1.X and v(p3.X)=p2.X
cv::Matx43d A(u1(0)*p1.at<double>(2,0)-p1.at<double>(0,0),
              u1(0)*p1.at<double>(2,1)-p1.at<double>(0,1),
              u1(0)*p1.at<double>(2,2)-p1.at<double>(0,2),
              u1(1)*p1.at<double>(2,0)-p1.at<double>(1,0),
              u1(1)*p1.at<double>(2,1)-p1.at<double>(1,1),
              u1(1)*p1.at<double>(2,2)-p1.at<double>(1,2),
              u2(0)*p2.at<double>(2,0)-p2.at<double>(0,0),
              u2(0)*p2.at<double>(2,1)-p2.at<double>(0,1),
              u2(0)*p2.at<double>(2,2)-p2.at<double>(0,2),
              u2(1)*p2.at<double>(2,0)-p2.at<double>(1,0),
              u2(1)*p2.at<double>(2,1)-p2.at<double>(1,1),
              u2(1)*p2.at<double>(2,2)-p2.at<double>(1,2));

cv::Matx41d B(p1.at<double>(0,3)-u1(0)*p1.at<double>(2,3),
              p1.at<double>(1,3)-u1(1)*p1.at<double>(2,3),
              p2.at<double>(0,3)-u2(0)*p2.at<double>(2,3),
              p2.at<double>(1,3)-u2(1)*p2.at<double>(2,3));

              // X contains the 3D coordinate of the reconstructed point
              cv::Mat X;
              // solve AX=B
              cv::solve(A,B,X,cv::DECOMP_SVD);
              return X;
              }


