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
using MatchMatrix = std::vector<std::vector<Matching>>;
using Map2D3D = std::map<int,Point3D2DMatch>;

class StructFromMotion{ 

  private:
  std::vector<cv::Mat>                    nImages;
  int                                     totalImages;
  CameraData                              matrixK;
  std::vector<Features>                   nFeaturesImages;
  std::vector<Pt2DAligned>                pts2DAligVec;
  std::vector<Matching>                   nFeaturesMatches;
  std::vector<std::vector<Matching>>      nFeatureMatchMatrix;
  std::vector<Point3D>                    nReconstructionCloud;
  std::vector<std::string>                nImagesPath;
  std::vector<cv::Mat_<double>>           projectionMatrices;
  cv::Ptr<cv::Feature2D>                   ptrFeature2D;
  cv::Ptr<cv::DescriptorMatcher>          matcherFlan;
  double                                  NN_MATCH_RATIO;


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

    ptrFeature2D = cv::ORB::create(5000.0);
    matcherFlan = cv::DescriptorMatcher::create("BruteForce-Hamming");
    NN_MATCH_RATIO = 0.8f;


  }

  //===============================================
  //CARGA DE IMAGENES
  //===============================================

  void recon( std::ifstream& file);

  void imagesLOAD(std::ifstream& file);

  //===============================================
  //FEATURE DETECTION AND EXTRACTION
  //===============================================

   Features obtenerFeatures(const cv::Mat& image);
   void extractFeatures();

  //===============================================
  //FEATURE MATCHING
  //===============================================

  Matching obtenerMatches(const Features& left,const Features& right);
 void matchFeatures();
  cv::Mat imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                        const cv::Mat& img2,const Keypoints& keypoints2,const Matching& matches);
  void matchingImShow(cv::Mat& matchImage);

  void guardarIdx(Matching& matches,Pt2DAligned& pt);

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

  cv::Mat findEssentialMatrix(const Points2f& leftPoints,const Points2f& rightPoints,
                                       const cv::Mat& cameraMatrix,cv::Mat& mask);

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

  int findHomographyInliers(const Features& f1,const Features& f2,const Matching& matches);

 std::map<float,ImagePair> best2Views();

  //===============================================
  //FUNCTION ALIGNED POINTS
  //===============================================

  void AlignedPointsFromMatch(const Features& left,const Features& right,const Matching& matches,Features& alignedL,Features& alignedR);
  void AlignedPoints(const Features& left,const Features& right,const Matching& matches, Features& alignedL, Features& alignedR,std::vector<int>& idLeftOrigen,std::vector<int>& idRightOrigen);



  //===============================================
  //FUNCTION CORRESPONDENCES 2D-3D
  //===============================================

  void map2D3D(const Features& left,const Features& right,const cv::Matx34f& P1,const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,const ImagePair imagePair,std::vector<Point3D>& pointcloud);
  void baseTriangulation();

  //===============================================
  //POINTCLOUD VISUALIZER
  //===============================================
  void visualizerPointCloud(const std::vector<Point3D>& pointcloud);

  //===============================================
  //INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
  //===============================================

  cv::Mat inverse(cv::Mat& matrix);
  double determinante(cv::Mat& relativeRotationCam);

 // std::vector<cv::Point3d> cloudPointsCoordinates(const std::vector<cv::Point3d> cloudpoint);


};//Fin class


