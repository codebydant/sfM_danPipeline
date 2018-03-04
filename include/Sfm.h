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

using FeaturesVector = std::vector<Features>;
using Pt2DAlignedVector = std::vector<Pt2DAligned>;

class StructFromMotion{ 

  private:
  std::vector<cv::Mat>               images;
  int                                totalImages;
  CameraData                         matrixK;
  std::vector<Features>              featuresImages;
  std::vector<Pt2DAligned>           pts2DAligVec;
  std::vector<Matches>               matchesImages;
  std::vector<Point3D>               pointcloud;
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

    ptrFeature2D = cv::xfeatures2d::SURF::create(2000.0);
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

   Features obtenerFeatures(const cv::Mat& image,const std::string& path);
   void loadFeatures(std::vector<cv::Mat>& imagesList,int& totalImages,
                     std::vector<Features>& ftsVector,std::vector<std::string>& imagePath);

  //===============================================
  //FEATURE MATCHING
  //===============================================

  Matches obtenerMatches(const cv::Mat& descriptors1,const cv::Mat& descriptors2);
  void loadMatches(const std::vector<Features>& ftsVec,std::vector<Matches>& matchesImages,
                   int& numImages);
  cv::Mat imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                        const cv::Mat& img2,const Keypoints& keypoints2,const MatchesVector& matches);
  void matchingImShow(cv::Mat& matchImage);

  void guardarIdx(Matches& matches,Pt2DAligned& pt);

  //===============================================
  //CONVERTION KEYPOINTS TO POINTS2D
  //===============================================

  void keypoints2F(Keypoints& keypoints, Points2f& points2D);

  //===============================================
  //CAMERA MATRIX
  //===============================================

  void getCameraMatrix(CameraData& intrinsics);

  //===============================================
  //ESSENTIAL MATRIX
  //===============================================

  cv::Mat_<double> findEssentialMatrix(const Points2f& leftPoints,const Points2f& rightPoints,
                                       cv::Mat_<double>& cameraMatrix,cv::Mat& mask);

  //===============================================
  //ROTATION AND TRASLATION MATRIX[R|t]
  //===============================================

  void cameraPose(const Points2f& points1,const Points2f& points2,const double& fx,const double cx,const double cy,
                  cv::Mat& rot,cv::Mat& tra,const cv::Mat& mask,const cv::Mat_<double>& essentialMatrix );

  //===============================================
  //PROJECTION MATRIX
  //===============================================

  void projection(cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam,
                  std::vector<cv::Mat_<double>>& matricesProVec,bool& status);


  //===============================================
  //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
  //===============================================

  bool CheckCoherentRotation(cv::Mat& R);

 int findHomographyInliers(const Features& f1,const Features& f2,const Matches& matches,Pt2DAligned& ptsAligned);

 void best2Views(std::vector<Features>& fts,std::map<float,idImagePair>& bestPair,std::vector<Pt2DAligned>& pts2dAlig);

  //===============================================
  //FUNCTION ALIGNED POINTS
  //===============================================

  void AlignedPoints(const Features& left,const Features& right,
                     const Matches& matches,std::vector<Pt2DAligned>& pts2dAlig);
  void AlignedPoints(const Features& left,const Features& right,const Matches& matches, Pt2DAligned& pts2dAlig);



  //===============================================
  //FUNCTION CORRESPONDENCES 2D-3D
  //===============================================

  void map2D3D(std::vector<cv::Mat_<double>>& ProjectionVector,std::vector<Pt2DAligned>& pts,CameraData& matrixK,bool& status, PointCloud& pointcloud);

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

 // std::vector<cv::Point3d> cloudPointsCoordinates(const std::vector<cv::Point3d> cloudpoint);


};//Fin class


