//***********************************************
//HEADERS
//***********************************************

#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/viz/viz3d.hpp"

//***********************************************
//ALIAS
//***********************************************

using Keypoints = std::vector<cv::KeyPoint>;
using MatchesVector = std::vector<cv::DMatch>;
using Points2f = std::vector<cv::Point2f>;
using Points3f = std::vector<cv::Point3f>;
using Points3d = std::vector<cv::Point3d>;

class StructFromMotion{ 

  //===============================================
  //ATRIBUTOS
  //===============================================

  cv::Mat_<cv::Vec3b> image1, image2;

  public:

  //===============================================
  //CONSTRUCTOR
  //===============================================
  StructFromMotion(){}
  StructFromMotion(cv::Mat& img1,cv::Mat& img2);

  //===============================================
  //CARGA DE IMAGENES
  //===============================================

  void recon( std::ifstream& file);
  int sizeTxtFile( std::ifstream& file);

  //===============================================
  //FEATURE DETECTION AND EXTRACTION
  //===============================================

  Keypoints obtenerKeypoints (cv::Mat& image);
  cv::Mat obtenerDescriptors (cv::Mat& image,Keypoints& keypoints);

  //===============================================
  //FEATURE MATCHING
  //===============================================

  MatchesVector obtenerMatches(cv::Mat& descriptors1,cv::Mat& descriptors2);
  cv::Mat imageMatching(cv::Mat& img1, Keypoints& keypoints1,
                        cv::Mat& img2, Keypoints& keypoints2,MatchesVector& matches);
  void matchingImShow(cv::Mat& matchImage);

  //===============================================
  //CONVERTION KEYPOINTS TO POINTS2F
  //===============================================

  Points2f keypoints2F(Keypoints& keypoints,MatchesVector& matches);

  //===============================================
  //CAMERA MATRIX
  //===============================================

  cv::Mat_<double> getCameraMatrix();

  //===============================================
  //ESSENTIAL MATRIX
  //===============================================

  cv::Mat_<double> findEssentialMatrix(Points2f& leftPoints,Points2f& rightPoints,
                                       cv::Mat_<double>& cameraMatrix,cv::Mat& mask);

  //===============================================
  //ROTATION AND TRASLATION MATRIX[R|t]
  //===============================================

  void cameraPose(Points2f& points1,Points2f& points2,double& fx,double cx,double cy,
                  cv::Mat& rot,cv::Mat& tra,cv::Mat& inliers,cv::Mat_<double>& essentialMatrix );

  //===============================================
  //PROJECTION MATRIX
  //===============================================

  void projection(const cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam,
                  cv::Mat& projection1, cv::Mat& projection2);


  //===============================================
  //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
  //===============================================

  bool CheckCoherentRotation(cv::Mat& R);

  //===============================================
  //FUNCTION ALIGNED POINTS
  //===============================================

  void AlignedPointsFromMatch(Keypoints& left,Keypoints& right, MatchesVector& matches,
                              Points2f& featuresLeftAligned,Points2f& featuresRightAligned);

  //===============================================
  //FUNCTION CORRESPONDENCES 2D-3D
  //===============================================



  //===============================================
  //POINTCLOUD VISUALIZER
  //===============================================
  void visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,
                            cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,Points3d& pointcloud);

  //===============================================
  //INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
  //===============================================

  cv::Mat inverse(cv::Mat& matrix);
  double determinante(cv::Mat& relativeRotationCam);

  Points3d cloudPointsCoordinates(const Points3d cloudpoint);


};//Fin class


