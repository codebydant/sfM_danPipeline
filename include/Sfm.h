//***********************************************
//HEADERS
//***********************************************

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "Structures.h"


class StructFromMotion{ 

  private:
  std::vector<cv::Mat>               images;
  int                                totalImages;
  std::vector<Features>              featuresImages;
  std::vector<Features>              featuresImagesBetter;
  std::vector<Matches>               matchesImages;
  std::vector<std::string>           imagesPath;
  std::vector<cv::Mat_<double>>      projectionMatrices;
  cv::Ptr<cv::Feature2D>             ptrFeature2D;
  cv::Ptr<cv::DescriptorMatcher>     matcherFlan;


  public:

  //===============================================
  //CONSTRUCTOR
  //===============================================

  StructFromMotion(){

    /*
    @ FLANNBASED = 1,
    @ BRUTEFORCE = 2,
    @ BRUTEFORCE_L1 = 3,
    @ BRUTEFORCE_HAMMING = 4,
    @ BRUTEFORCE_HAMMINGLUT = 5,
    @ BRUTEFORCE_SL2 = 6
    */

    ptrFeature2D = cv::xfeatures2d::SURF::create(2300.0);
    matcherFlan = cv::DescriptorMatcher::create("FlannBased");

  }

  //===============================================
  //CARGA DE IMAGENES
  //===============================================

  void recon( std::ifstream& file);

  void imagesLOAD(std::ifstream& file,std::vector<cv::Mat>& imageSet,int& numImages,
                  std::vector<std::string>& textFile);

  //===============================================
  //FEATURE DETECTION AND EXTRACTION
  //===============================================

   Features obtenerFeatures(cv::Mat& image,std::string& path);
   void loadFeatures(std::vector<cv::Mat>& imagesList,int& totalImages,
                     std::vector<Features>& ftsVector,std::vector<std::string>& imagePath);

  //===============================================
  //FEATURE MATCHING
  //===============================================

  Matches obtenerMatches(cv::Mat& descriptors1,cv::Mat& descriptors2);
  void loadMatches(std::vector<Features>& ftsVec,std::vector<Matches>& matchesImages,
                   int& numImages);
  cv::Mat imageMatching(cv::Mat& img1, Keypoints& keypoints1,
                        cv::Mat& img2, Keypoints& keypoints2,MatchesVector& matches);
  void matchingImShow(cv::Mat& matchImage);

  //===============================================
  //CONVERTION KEYPOINTS TO POINTS2D
  //===============================================

  void keypoints2F(Keypoints& keypoints, Points2d& points2D);

  //===============================================
  //CAMERA MATRIX
  //===============================================

  cv::Mat_<double> getCameraMatrix();

  //===============================================
  //ESSENTIAL MATRIX
  //===============================================

  cv::Mat_<double> findEssentialMatrix(Points2d& leftPoints,Points2d& rightPoints,
                                       cv::Mat_<double>& cameraMatrix,cv::Mat& mask);

  //===============================================
  //ROTATION AND TRASLATION MATRIX[R|t]
  //===============================================

  void cameraPose(Points2d& points1,Points2d& points2,double& fx,double cx,double cy,
                  cv::Mat& rot,cv::Mat& tra,cv::Mat& mask,cv::Mat_<double>& essentialMatrix );

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

  void AlignedPoints( Features& left, Features& right,Matches& matches);

  void AlignedPointsFromMatch( Features& left, Features& right,
                               Matches& matches,Features& leftAligned,Features& rightAligned);

  //===============================================
  //FUNCTION CORRESPONDENCES 2D-3D
  //===============================================



  //===============================================
  //POINTCLOUD VISUALIZER
  //===============================================
  void visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,
                            cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,Points3f& pointcloud);

  //===============================================
  //INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
  //===============================================

  cv::Mat inverse(cv::Mat& matrix);
  double determinante(cv::Mat& relativeRotationCam);

  Points3d cloudPointsCoordinates(const Points3d cloudpoint);


};//Fin class


