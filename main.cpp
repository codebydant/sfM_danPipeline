/*********************************
           HEADERS
**********************************/


#include <iostream>
#include <fstream>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/viz/viz3d.hpp"
#include <eigen3/Eigen/Dense>
#include <string>
#include <mutex>

#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/openni_grabber.h"
#include "pcl/console/parse.h"
#include "pcl/common/transforms.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"


/*********************************
        FUNCIONES
**********************************/

static void help();
int frameNum(cv::VideoCapture &inputVideo);
cv::Mat cargarFrame(cv::VideoCapture &inputVideo);
cv::Mat obtenerMatrixH(cv::Mat& image1,cv::Mat& image2);
cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2, cv::Mat& invMatrixH);
cv::Mat imageMatching(cv::Mat& image1,cv::Mat& image2);
cv::Mat detMatrixE(cv::Mat& matrixE12,cv::Mat& matrixE23);
std::vector<cv::Point2f> keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches);
std::vector<cv::KeyPoint> obtenerKeypoints(cv::Mat& image);
std::vector<cv::DMatch> obtenerMatches(cv::Mat& image1,
                                       cv::Mat& image2,
                                       std::vector<cv::KeyPoint>& keypoints1,
                                       std::vector<cv::KeyPoint>& keypoints2);
cv::Mat triangulate(const cv::Mat& p1,const cv::Mat& p2, cv::Vec2d& u1, cv::Vec2d& u2);
std::vector<cv::DMatch> thresholdGoodMatches(cv::Mat& descriptors,std::vector<cv::DMatch>& matches);
double TriangulatePoints(const std::vector<cv::KeyPoint>& pt_set1,
                         const std::vector<cv::KeyPoint>& pt_set2,const cv::Mat&Kinv,
                         const cv::Matx34d& P,const cv::Matx34d& P1,std::vector<cv::Point3d>& pointcloud,
                         const cv::Mat& K);
cv::Mat LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1);

cv::Mat TriangulatePoints2(const std::vector<cv::KeyPoint>& pt_set1,
                         const std::vector<cv::KeyPoint>& pt_set2,const cv::Mat&Kinv,
                         const cv::Matx34d& P,const cv::Matx34d& P1,
                         const cv::Mat& K);
cv::Vec3d triangulate2(const cv::Mat& p1,const cv::Mat& p2, cv::Vec2d& u1, cv::Vec2d& u2);

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat){
       pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);


   // calibration parameters

       float const fx_d = 409;
       float const fy_d = 408;
       float const cx_d = 237;
       float const cy_d = 171;

       unsigned char* p = depthMat.data;
       for (int i = 0; i<depthMat.rows; i++)
       {
           for (int j = 0; j < depthMat.cols; j++)
           {
               float z = static_cast<float>(*p);
               pcl::PointXYZ point;
               point.z = 0.001 * z;
               point.x = point.z*(j - cx_d)  / fx_d;
               point.y = point.z *(cy_d - i) / fy_d;
               ptCloud->points.push_back(point);
               ++p;
           }
       }
       ptCloud->width = (int)depthMat.cols;
       ptCloud->height = (int)depthMat.rows;

       return ptCloud;

   }
cv::Mat_<double> LinearLSTriangulation4(cv::Point2d u,cv::Matx34d P,cv::Point2d u1,cv::Matx34d P1);

std::vector<cv::Point3d> LinearLSTriangulation5(cv::Point2d u,cv::Matx34d P,
                                                cv::Point2d u1,cv::Matx34d P1){
//build A matrix
cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

//build B vector
cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),-(u.y*P(2,3)-P(1,3)),-(u1.x*P1(2,3)-P1(0,3)),-(u1.y*P1(2,3)-P1(1,3)));

//solve for X
std::vector<cv::Point3d> X;
cv::solve(A,B,X,cv::DECOMP_SVD);
return X;
}



/*********************************
      FUNCION PRINCIPAL-MAIN
**********************************/


int main(int argc, char **argv ){


   cv::Mat image1,image2,matchImage;
   cv::Mat temp_img2;
   std::vector<cv::Point3d> temp_result;

   std::ifstream file("temple/list.txt");
   std::string frame1,frame2;
   std::vector<cv::KeyPoint> keypoints1;
   std::vector<cv::KeyPoint> keypoints2;

   int x=1;     
   std::vector<cv::Point2f>points1,points2;
   cv::Mat_<double> pointCloudOpenCV;
/*
  // cv::Mat cameraMatrix;
   cv::Mat cameraDistCoeffs;
   cv::FileStorage fs("camera-calibration-data.xml", cv::FileStorage::READ);
  // fs["Camera_Matrix"] >> cameraMatrix;
   fs["Distortion_Coefficients"] >> cameraDistCoeffs;
   //cv::Matx33d cMatrix(cameraMatrix);
   std::vector<double> cMatrixCoef(cameraDistCoeffs);
   std::cout <<"Vector distortion coeff: "<< std::endl;
   std::cout << "[ ";
   for(size_t n=0;n<cMatrixCoef.size();n++){

       std::cout << cMatrixCoef.at(n)<< ",";
     }
   std::cout <<"]" << std::endl;
   */
/*
   Eigen::MatrixXd invMatrixK,invMatrixKTranspose;
   Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
                                   Eigen::Dynamic,
                                   Eigen::RowMajor> > eigenMatrixK((double *)cameraMatrix.data,3,3);

   invMatrixK = eigenMatrixK.inverse();
   invMatrixKTranspose = invMatrixK.transpose();
   // create an OpenCV Mat header for the Eigen data:
   cv::Mat invMatrixKOpenCV(invMatrixKTranspose.rows(),
                                        invMatrixKTranspose.cols(),
                                        CV_64FC1,invMatrixKTranspose.data());


*/
   cv::Mat matrixE12;
   std::vector<cv::Point2d> points_image1;
   std::vector<cv::Point2d> points_image2;
   std::vector<cv::Point2d> inlierPtsImage1;
   std::vector<cv::Point2d> inlierPtsImage2;
   std::vector<cv::Point2d> points1Undis;
   std::vector<cv::Point2d> points2Undis;
   cv::Mat relativeRotationCam;
   cv::Mat relativeTranslaCam;


int cont =0;

  //   while(cont <7){

      //    if (x==1) {
/*
              std::getline(file, frame1);
              std::cout << frame1 << std::endl;
              std::getline(file, frame2);
              std::cout << frame2 << std::endl;
*/

              image1 = cv::imread("soup1.jpg",0);
              image2 = cv::imread("soup2.jpg",0);
              GaussianBlur(image1,image1, cv::Size(7,7),1.5,1.5);
              GaussianBlur(image2,image2, cv::Size(7,7),1.5,1.5);

              /**************************************************
                  CÁLCULO DE LA MATRIZ INVERSA DE LA MATRIZ H
              ***************************************************/
              keypoints1 = obtenerKeypoints(image1);
              keypoints2 = obtenerKeypoints(image2);

              std::vector<cv::DMatch> matches = obtenerMatches(image1,
                                                               image2, keypoints1,keypoints2);
              points1= keypoints2F( keypoints1, matches);
              points2= keypoints2F( keypoints2, matches);

             // std::cout <<"points1 : "<< "\n" << points1 << std::endl;
//              std::cout <<"points2 : "<< "\n" << points2 << std::endl;

/*
              for (int i = 0; i < image1.cols; i++){
                    for (int j = 0; j < image1.rows; j++) {

                          points_image1.push_back(image1.at<cv::Point2d>(cv::Point(i, j)));
                          points_image2.push_back(image2.at<cv::Point2d>(cv::Point(i, j)));
                     }
                  }
*/



           //   double focalLengthX,focalLengthY,focalength;
              //double cx,cy;
             // focalength = 800;
              double fx = 409;
              double fy = 408;
              double cx = 237;
              double cy = 171;
           //   double f =  focalength;

              cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx,0,cx,
                                                               0,fy,cy,
                                                               0,0,1);
              cv::Matx33d cMatrix(cameraMatrix);

             // focalLengthX = cameraMatrix.at<double>(0,0);
              //focalLengthY = cameraMatrix.at<double>(1,1);

            //  cx= cameraMatrix.at<double>(0,2);
            //  cy = cameraMatrix.at<double>(1,2);


              // Find the essential between image 1 and image 2
 cv::Mat inliers;
              cv::Mat matrixFundamental = cv::findFundamentalMat(points1,points2,cv::FM_RANSAC,0.1,0.99,inliers);
              std::cout<< "total inliers" << inliers.size()<<std::endl;

              matrixE12 = cameraMatrix.t()*matrixFundamental*cameraMatrix;
               std::cout <<"Matrix Essential us: "<< "\n" << matrixE12 << std::endl;

cv::Mat h;
            //  matrixE12 = cv::findEssentialMat(points1, points2,cameraMatrix,cv::RANSAC,0.99, 0.1,inliers);
matrixE12 = (cv::Mat_<double>(3,3) <<

0.001905132905183011, -0.3512859268097117, 0.01888495989911259,
 0.2692589029874893, -0.006758526413464595, -0.6536178451838158,
 -0.01513595230914446, 0.6133743732940988, -0.004287826651009312);
          matchImage= imageMatching( image1,image2);
              cv::imshow("matching",matchImage);
              cv::waitKey(0);


              std::cout <<"Matrix Essential: "<< "\n" << matrixE12 << std::endl;

              // recover relative camera pose from essential matrix

              cv::recoverPose(matrixE12,points1, points2,
                              relativeRotationCam, relativeTranslaCam,fx,cv::Point2d(cx,cy),inliers);


              std::cout <<"Matriz de rotación relativa 3x3[R]: "<< "\n" << relativeRotationCam << std::endl;
              std::cout <<"Vector de traslación relativo 3x1[t]: "<< "\n" << relativeTranslaCam << std::endl;

              /******************************
                MATRIZ DE PROYECCIÓN HOMOGENÉA [R|t]
              ******************************/

              cv::Mat projection2(3, 4, CV_64F,cv::Scalar(0.0));

              std::cout << "First Projection matrix=" << projection2 << std::endl;
              relativeRotationCam.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
              std::cout << "relative rotation Projection matrix=" << projection2 << std::endl;

              relativeTranslaCam.copyTo(projection2.colRange(3, 4));
              std::cout << "relative traslation Projection matrix=" << projection2 << std::endl;
              /*
                              |     |   |
                projection2 = |  R  | t |
                              |_____|___|
              */




              // compose generic projection matrix
              cv::Mat projection1(3, 4, CV_64F, cv::Scalar(0.0));
              std::cout << "projection1" << projection1 << std::endl;

              cv::Mat diag(cv::Mat::eye(3, 3, CV_64F)); // ---> Crea una matriz indentidad
                                                        /*
                                                                 |1 0 0|
                                                          diag = |0 1 0|
                                                                 |0 0 1|
                                                        */

              std::cout << "diag" << diag << std::endl;

              diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
              std::cout << "projection1 final: " << projection1 << std::endl;

/*

              cv::imshow("inliers",matchImage);
              cv::waitKey(0);
*/


              for (int i=0;i<inliers.rows;i++) {

                      inlierPtsImage1.push_back(cv::Point2d(points1[i].x,points1[i].y));
                      inlierPtsImage2.push_back(cv::Point2d(points2[i].x,points2[i].y));
              }



              std::vector<cv::Point3d> cloud;


            //  cv::undistortPoints(inlierPtsImage1, points1Undis,cameraMatrix, cameraDistCoeffs);
             // cv::undistortPoints(inlierPtsImage2, points2Undis,cameraMatrix, cameraDistCoeffs);

              for(unsigned int n=0;n<inlierPtsImage1.size();n++){

                 pointCloudOpenCV.push_back(LinearLSTriangulation4(inlierPtsImage1.at(n),projection1,
                                                           inlierPtsImage2.at(n),projection2));
                cloud.push_back(cv::Point3d(pointCloudOpenCV(0),pointCloudOpenCV(1),pointCloudOpenCV(2)));
                // std::cout << "pointcloudOpencv: " << cloud << std::endl;
                // cv::waitKey(0);
               }
/*
              cv::Mat cloud2(cloud,CV_64FC4);
              */
/*
              for (unsigned int i=0; i<keypoints1.size(); i++) {
              //convert to normalized homogeneous coordinates
                 cv::Point2f kp = keypoints1[i].pt;
                 cv::Point3d u(kp.x,kp.y,1.0);
                 cv::Mat_<double> um = invMatrixKOpenCV * cv::Mat_<double>(u);
                 u = um.at<cv::Point3d>(0);
                 cv::Point2f kp1 = keypoints2[i].pt;
                 cv::Point3d u1(kp1.x,kp1.y,1.0);
                 cv::Mat_<double> um1 = invMatrixKOpenCV * cv::Mat_<double>(u1);
                 u1 = um1.at<cv::Point3d>(0);
                 //triangulate
                cv::Mat_<double> X = LinearLSTriangulation(u,projection1,u1,projection2);

                 //store 3D point
                 cloud.push_back(cv::Point3d(X(0),X(1),X(2)));
              }

*/
              /*
              temp_result = cloud;
              temp_img2 = image2;
              cont +=1;
              x=2;
*/
              //create inliers input point vector for triangulation

     //     }
     /*
     else if(x==2){

              keypoints1.clear();
              keypoints2.clear();


              image1 = temp_img2;//Frame 2
              std::getline(file, frame2);
              std::cout << frame2 << std::endl;
              image2 = cv::imread(frame2,CV_LOAD_IMAGE_COLOR);//Frame 3
              GaussianBlur(image2,image2, cv::Size(7,7),1.5,1.5);

             // cv::imshow("frame2",image1);
             // cv::imshow("frame3",image2);
             // cv::waitKey(0);

              std::vector<cv::Point2d> points_image3;


              keypoints1 = obtenerKeypoints(image1);
              keypoints2 = obtenerKeypoints(image2);


              std::vector<cv::DMatch> matches = obtenerMatches(image1,image2, keypoints1,keypoints2);
              points1= keypoints2F( keypoints1, matches);
              points2= keypoints2F( keypoints2, matches);

              for (int i = 0; i < image2.cols; i++){
                    for (int j = 0; j < image2.rows; j++) {

                          points_image3.push_back(image2.at<cv::Point2d>(cv::Point(i, j)));

                     }
                  }


              double focalLengthX,focalLengthY;
              double cx,cy;

              focalLengthX = cameraMatrix.at<double>(0,0);
              focalLengthY = cameraMatrix.at<double>(1,1);

              cx= cameraMatrix.at<double>(0,2);
              cy = cameraMatrix.at<double>(1,2);


              // Find the essential between image 2 and image 3
              cv::Mat inliers;

              cv::Mat matrixE23 = cv::findEssentialMat(points1, points2,
                                                       focalLengthX,cv::Point2d(cx,cy),cv::RANSAC,0.9, 1.0,inliers);

              cv::Mat essentialTotal = detMatrixE(matrixE12,matrixE23);

              std::cout <<"Matrix Essential Total: "<< "\n" << essentialTotal << std::endl;

              // recover relative camera pose from essential matrix

              cv::recoverPose(essentialTotal,points1, points2,
                              relativeRotationCam, relativeTranslaCam,focalLengthX,cv::Point2d(cx,cy),inliers);

              std::cout <<"Matriz de rotación relativa 3x3[R]: "<< "\n" << relativeRotationCam << std::endl;
              std::cout <<"Vector de traslación relativo 3x1[t]: "<< "\n" << relativeTranslaCam << std::endl;

              // compose projection matrix from R,T
              cv::Mat projection2(3, 4, CV_64F);
              relativeRotationCam.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
              relativeTranslaCam.copyTo(projection2.colRange(3, 4));

              // compose generic projection matrix
              cv::Mat projection1(3, 4, CV_64F, 0.);
              cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
              diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));


              std::cout << "First Projection matrix=" << projection1 << std::endl;
              std::cout << "Second Projection matrix=" << projection2 << std::endl;


              std::vector<cv::Point2d> inlierPtsImage3;

              for (int i=0;i<inliers.rows;i++) {

                     // inlierPtsImage1.push_back(cv::Point2d(points_image2[i].x,points_image1[i].y));
                      inlierPtsImage3.push_back(cv::Point2d(points_image3[i].x,points_image3[i].y));
              }


              std::vector<cv::Point2d> points3Undis;
              cv::undistortPoints(inlierPtsImage3, points3Undis,cameraMatrix, cameraDistCoeffs);


              for(unsigned int n=0;n<points2Undis.size();n++){

                 pointCloudOpenCV.push_back(LinearLSTriangulation4(points2Undis.at(n),projection1,
                                                           points3Undis.at(n),projection2));
                  temp_result.push_back(cv::Point3d(pointCloudOpenCV(0),pointCloudOpenCV(1),pointCloudOpenCV(2)));
               }

              temp_img2 = image2;
              matrixE12 = essentialTotal;

              cont +=1;
              x = 2;


              }

            }
 //    std::cout << "cloud points:" << temp_result << std::endl;
 */
/*
             cv::Mat X = cv::Mat(temp_result.size(),temp_result.size(),CV_8UC1);

             std::memcpy(X.data, temp_result.data(), temp_result.size()*sizeof(uchar));
*/



                            // Create a viz window
                            cv::viz::Viz3d visualizer("Viz window");

                            cv::viz::WCoordinateSystem ucs(0.05);
                            ucs.setRenderingProperty(cv::viz::LINE_WIDTH,0.05);

                            // Add line to represent (1,1,1) axis
                         //  cv::viz::WLine axis(cv::Point3f(0,0,0), cv::Point3f(1.0f,1.0f,1.0f));
                           // axis.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);

                            // Create a virtual camera
                            cv::viz::WCameraPosition cam1(cMatrix, 0.5,cv::viz::Color::white());
                            cam1.setRenderingProperty(cv::viz::LINE_WIDTH,2);


                            cv::viz::WCameraPosition cam2(cMatrix,0.5,cv::viz::Color::white());
                            cam2.setRenderingProperty(cv::viz::LINE_WIDTH,2);
                            // Add the virtual camera to the environment


                           cv::viz::WCloud point3d(cloud, cv::viz::Color::green());

                            point3d.setRenderingProperty(cv::viz::POINT_SIZE, 3.0);

                            visualizer.setBackgroundColor(cv::viz::Color::gray());
                            visualizer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
                            visualizer.showWidget("Line Widget", ucs);
                            visualizer.showWidget("Camera1", cam1);
                            visualizer.showWidget("Camera2", cam2);
                            visualizer.showWidget("Point3D", point3d);

                            cv::Affine3d pose(relativeRotationCam,relativeTranslaCam);

                            visualizer.setWidgetPose("Camera2", pose);

                            // visualization loop
                            while(cv::waitKey(0) && !visualizer.wasStopped()){

                                visualizer.spin();
                            }




/*

              pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudPCL;
              pointcloudPCL = MatToPoinXYZ(pointCloudOpenCV);

              pcl::visualization::PCLVisualizer viewer ("Pointcloud PCL",true);

              // Define R,G,B colors for the point cloud
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(pointcloudPCL,
                                                                                                        0, 255, 0);


           //   Eigen::Affine3f pcl::visualization::PCLVisualizer::getViewerPose;
              // We add the point cloud to the viewer and pass the color handler
              viewer.addPointCloud (pointcloudPCL, source_cloud_color_handler, "original_cloud");
              viewer.addCoordinateSystem (0.02, "cloud", 0);
              viewer.setBackgroundColor(0, 0, 0, 0); // Setting background to a dark grey
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                                    3, "original_cloud");

              while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
                viewer.spinOnce ();
                           }
                           */
/*
*/


//}

//cvWaitKey(0);
   return 0;

  }//end main function


/*********************************
        FUNCIONES-CUERPO
**********************************/
cv::Mat detMatrixE(cv::Mat& matrixE12,cv::Mat& matrixE23){
cv::Mat matrixE_n_esima;
   matrixE_n_esima = matrixE12*matrixE23;

  return matrixE_n_esima;
}

int frameNum(cv::VideoCapture &inputVideo){

  int framesNumber=0;
  framesNumber = static_cast<int>(inputVideo.get(CV_CAP_PROP_FRAME_COUNT));
  return framesNumber;

}

cv::Mat cargarFrame(cv::VideoCapture &inputVideo){

   cv::Mat frame,image,image_resized;
   inputVideo.read(frame);
   /*
   if(frame.channels()==3 || frame.channels()==1){
      cv::cvtColor(frame,frame, cv::COLOR_BGR2GRAY);
    }
   */
   image = frame.clone();
   cv::resize(image,image_resized,cv::Size(),0.75,0.75);
   GaussianBlur(image_resized,image, cv::Size(7,7),1.5,1.5);
   double position = inputVideo.get(CV_CAP_PROP_POS_FRAMES );

   std::ostringstream strs;
   strs << position;
   std::string str2 = strs.str();
   std::string str1= "Frame";

   str1 +=str2;
   cv::putText(image,str1, cv::Point(50,500), cv::FONT_HERSHEY_SIMPLEX ,1.0, 0, 4);
   return image;
}

cv::Mat imageMatching(cv::Mat& image1,cv::Mat& image2){
  cv::Mat descriptors1,descriptors2,matchImage;
  std::vector<cv::KeyPoint> keypoints1,keypoints2;

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);
  cv::FlannBasedMatcher matcher;

  std::vector<cv::DMatch> matches,good_matches; 

  ptrFeature2D->detect(image1,keypoints1);
  ptrFeature2D->detect(image2,keypoints2);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  matcher.match(descriptors1,descriptors2,matches);
  good_matches = thresholdGoodMatches(descriptors1,matches);

  cv::drawMatches(image1,keypoints1,image2,keypoints2,good_matches,matchImage,cv::Scalar::all(-1),
  cv::Scalar::all(-1),std::vector<char>(),2);

  return matchImage;
}

cv::Mat obtenerMatrixH(cv::Mat& image1,cv::Mat& image2){

  cv::Mat descriptors1,descriptors2;
  std::vector<cv::KeyPoint> keypoints1,keypoints2;

  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);

  ptrFeature2D->detect(image1,keypoints1);
  ptrFeature2D->detect(image2,keypoints2);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches,good_matches;
  matcher.match(descriptors1,descriptors2,matches);

  good_matches = thresholdGoodMatches(descriptors1,matches);

  cv::Mat matchImage;

  cv::drawMatches(image1,keypoints1,image2,keypoints2,good_matches,matchImage,cv::Scalar::all(-1),
  cv::Scalar::all(-1),std::vector<char>(),2);

  // 1st image is the destination image and the 2nd image is the src image

  std::vector<cv::Point2f> obj; //1st image
  std::vector<cv::Point2f> obj2;//2nd image

  for(size_t i=0;i<good_matches.size();i++)
  {
      obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
      obj2.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }

  cv::Mat matrixH(3,3,CV_32FC3);
  matrixH = cv::findHomography(obj,obj2,cv::RANSAC,1);

  std::cout << "Matrix H: "<< "\n" << matrixH << std::endl;

  return matrixH;
}

cv::Mat imageStitching(cv::Mat& image1,cv::Mat& image2,  cv::Mat& invMatrixH){

  cv::Mat result;
  cv::warpPerspective(image2,result,invMatrixH,cv::Size(image1.cols,image1.rows),  cv::INTER_CUBIC);

    for (int i = 0; i < image1.cols; i++){
          for (int j = 0; j < image1.rows; j++) {

		cv::Vec3b color_im1 = image1.at<cv::Vec3b>(cv::Point(i, j));
		cv::Vec3b color_im2 = result.at<cv::Vec3b>(cv::Point(i, j));
		if (cv::norm(color_im1) == 0){
			image1.at<cv::Vec3b>(cv::Point(i, j)) = color_im2;
			}
	   }
	}

  return image1;

}

static void help(){
   std::cout <<  "------------------------------------------------------------------------------------"
                 "This program shows the multiview reconstruction capabilities using the "
                 "OpenCV Library."
                 "It reconstruct a scene from a set of 2D images"
                 "Usage:"
                 "3D_recons <path_to_file> <f> <cx> <cy>"
                 "where: path_to_file is the file absolute path into your system which contains"
                 "the list of images to use for reconstruction."
                 "f  is the focal lenght in pixels. "
                 "cx is the image principal point x coordinates in pixels. "
                 "cy is the image principal point y coordinates in pixels."
            "------------------------------------------------------------------------------------" <<std::endl;
}

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


cv::Vec3d triangulate2(const cv::Mat& p1,const cv::Mat& p2, cv::Vec2d& u1, cv::Vec2d& u2){
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
              cv::Vec3d X;
              // solve AX=B
              cv::solve(A,B,X,cv::DECOMP_SVD);
              return X;
              }



std::vector<cv::KeyPoint> obtenerKeypoints(cv::Mat& image){

  std::vector<cv::KeyPoint> keypoints;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);
  ptrFeature2D->detect(image,keypoints);

  return keypoints;
}

std::vector<cv::DMatch> obtenerMatches(cv::Mat& image1,cv::Mat& image2,
                       std::vector<cv::KeyPoint>& keypoints1,std::vector<cv::KeyPoint>& keypoints2){

  cv::Mat descriptors1,descriptors2;
  cv::Ptr<cv::Feature2D> ptrFeature2D = cv::xfeatures2d::SURF::create(1000.0);

  ptrFeature2D->compute(image1,keypoints1,descriptors1);
  ptrFeature2D->compute(image2,keypoints2,descriptors2);

  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches,good_matches;
  matcher.match(descriptors1,descriptors2,matches);

  std::cout << "Total matches: " << matches.size() << std::endl;

  good_matches = thresholdGoodMatches(descriptors1,matches);

  std::cout << "Total good matches: " << good_matches.size() << std::endl;

  // 1st image is the destination image and the 2nd image is the src image

  std::vector<cv::Point2f> obj; //1st image
  std::vector<cv::Point2f> obj2;//2nd image

  for(size_t i=0;i<good_matches.size();i++)
  {
      obj.push_back(keypoints1[good_matches[i].queryIdx].pt);
      obj2.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }

  return good_matches;
}

std::vector<cv::Point2f> keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches){

  std::vector<cv::Point2f> points;
  for (std::vector<cv::DMatch>::const_iterator it= matches.begin();it!= matches.end(); ++it){

      // Get the position of left keypoints
                   float x= keypoints[it->queryIdx].pt.x;
                   float y= keypoints[it->queryIdx].pt.y;
                   points.push_back(cv::Point2f(x,y));                 
            }
  return points;
}

std::vector<cv::DMatch> thresholdGoodMatches(cv::Mat& descriptors,std::vector<cv::DMatch>& matches) {

        double max_dist=0.8;
        std::vector<cv::DMatch> good_matches;

        double min_dist = 100;

        for(int i=0;i<descriptors.rows;i++){

           double dist = matches[i].distance;
           if(dist<min_dist) {
                min_dist = dist;

           }if(dist > max_dist){
               max_dist = dist;
           }
        }

      for(int i=0;i<matches.size();i++){

        if(matches[i].distance <= min_dist*3){

              good_matches.push_back(matches[i]);
         }
      }

      return good_matches;
    }

cv::Mat LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1){
//build A matrix
cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

//build B vector
cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),-(u.y*P(2,3)-P(1,3)),-(u1.x*P1(2,3)-P1(0,3)),-(u1.y*P1(2,3)-P1(1,3)));

//solve for X
cv::Mat_<double> X;
cv::solve(A,B,X,cv::DECOMP_SVD);
return X;
}

double TriangulatePoints(const std::vector<cv::KeyPoint>& pt_set1,
                         const std::vector<cv::KeyPoint>& pt_set2,const cv::Mat&Kinv,
                         const cv::Matx34d& P,const cv::Matx34d& P1,std::vector<cv::Point3d>& pointcloud,
                         const cv::Mat& K){
        std::vector<double> reproj_error;
        for (unsigned int i=0; i<pt_set1.size(); i++) {
        //convert to normalized homogeneous coordinates
           cv::Point2f kp = pt_set1[i].pt;
           cv::Point3d u(kp.x,kp.y,1.0);
           cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
           u = um.at<cv::Point3d>(0);
           cv::Point2f kp1 = pt_set2[i].pt;
           cv::Point3d u1(kp1.x,kp1.y,1.0);
           cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
           u1 = um1.at<cv::Point3d>(0);
           //triangulate
           cv::Mat_<double> X = LinearLSTriangulation(u,P,u1,P1);
           //calculate reprojection error
           cv::Mat_<double> xPt_img = K * cv::Mat(P1) * X;
           cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
           reproj_error.push_back(cv::norm(xPt_img_-kp1));
           //store 3D point
           pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));
        }
        //return mean reprojection error
        cv::Scalar me = cv::mean(reproj_error);
        return me[0];
}


cv::Mat TriangulatePoints2(const std::vector<cv::KeyPoint>& pt_set1,
                         const std::vector<cv::KeyPoint>& pt_set2,const cv::Mat&Kinv,
                         const cv::Matx34d& P,const cv::Matx34d& P1,
                         const cv::Mat& K){
  std::vector<cv::Point3d> pointcloud;
        std::vector<double> reproj_error;
        cv::Mat_<double> X ;
        for (unsigned int i=0; i<pt_set1.size(); i++) {
        //convert to normalized homogeneous coordinates
           cv::Point2f kp = pt_set1[i].pt;
           cv::Point3d u(kp.x,kp.y,1.0);
           cv::Mat_<double> um = Kinv * cv::Mat_<double>(u);
           u = um.at<cv::Point3d>(0);
           cv::Point2f kp1 = pt_set2[i].pt;
           cv::Point3d u1(kp1.x,kp1.y,1.0);
           cv::Mat_<double> um1 = Kinv * cv::Mat_<double>(u1);
           u1 = um1.at<cv::Point3d>(0);
           //triangulate
            X = LinearLSTriangulation(u,P,u1,P1);
           //calculate reprojection error
           cv::Mat_<double> xPt_img = K * cv::Mat(P1) * X;
           cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
           reproj_error.push_back(cv::norm(xPt_img_-kp1));
           //store 3D point
           pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));
        }
        //return mean reprojection error
        //cv::Scalar me = cv::mean(reproj_error);
        return X;
}


cv::Mat_<double> LinearLSTriangulation4(cv::Point2d u,cv::Matx34d P,cv::Point2d u1,cv::Matx34d P1){
//build A matrix
cv::Matx43d A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2));

//build B vector
cv::Matx41d B(-(u.x*P(2,3)-P(0,3)),-(u.y*P(2,3)-P(1,3)),-(u1.x*P1(2,3)-P1(0,3)),-(u1.y*P1(2,3)-P1(1,3)));

//solve for X
cv::Mat_<double> X;
cv::solve(A,B,X,cv::DECOMP_SVD);
return X;
}
