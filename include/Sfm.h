//----------------------------------------------
//HEADERS
//----------------------------------------------
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

using Keypoints = std::vector<cv::KeyPoint>;
using MatchesVector = std::vector<cv::DMatch>;
using Points2f = std::vector<cv::Point2f>;

class StructFromMotion{ 

  cv::Mat_<cv::Vec3b> image1, image2;

  public:

  //----------------------------------------------
  //CONSTRUCTOR
  //----------------------------------------------
  StructFromMotion(){}
  StructFromMotion(cv::Mat& img1,cv::Mat& img2);

  //----------------------------------------------
  //PIPELINE FUNCTIONS
  //----------------------------------------------

  void recon( std::ifstream& file);

  int sizeTxtFile( std::ifstream& file);

  Keypoints obtenerKeypoints (cv::Mat& image);
  cv::Mat obtenerDescriptors (cv::Mat& image,Keypoints& keypoints);

  MatchesVector obtenerMatches(cv::Mat& descriptors1,cv::Mat& descriptors2);

  cv::Mat imageMatching(cv::Mat& img1, Keypoints& keypoints1,cv::Mat& img2, Keypoints& keypoints2,
                        MatchesVector& matches);

  void matchingImShow(cv::Mat& matchImage);

  Points2f keypoints2F(Keypoints& keypoints,MatchesVector& matches);

  cv::Mat_<double> getCameraMatrix();

 cv::Mat_<double> findEssentialMatrix( Points2f& leftPoints,Points2f& rightPoints,cv::Mat_<double>& cameraMatrix,cv::Mat& mask);

  void cameraPose(Points2f& points1,Points2f& points2,
                  double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,
                  cv::Mat& inliers,cv::Mat_<double>& essentialMatrix );

  void projection(const cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam,
                  cv::Mat_<double>& projection1, cv::Mat_<double>& projection2);

 // std::vector<cv::Point3d> triangulation(std::vector<cv::Point2f>& points1,
                                     //    std::vector<cv::Point2f>& points2,
                                     //    cv::Mat_<double>& projection1,
                                     //    cv::Mat_<double>& projection2,cv::Mat& inliers,cv::Mat_<double>& cameraMatrix);

  void visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,
                            cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,cv::Mat& pointcloud);

  void setConstructor(cv::Mat& img1,cv::Mat& img2);

  //----------------------------------------------
  //INVERSE MATRIX FUNCTION EIGEN
  //----------------------------------------------

  cv::Mat inverse(cv::Mat& matrix);

  double determinante(cv::Mat& relativeRotationCam);

  //----------------------------------------------
  //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
  //----------------------------------------------

  bool CheckCoherentRotation(cv::Mat_<double>& R); 

  //----------------------------------------------
  //FUNCTION ALIGNED POINTS
  //----------------------------------------------

  void AlignedPointsFromMatch(Keypoints& left,Keypoints& right, MatchesVector& matches, Points2f& featuresLeftAligned,Points2f& featuresRightAligned);

  void Find2D3DCorrespondences(int working_view,std::vector<cv::Point3f>& ppcloud,
          Points2f& imgPoints);




};//Fin class


