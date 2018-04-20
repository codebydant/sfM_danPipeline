//***********************************************
//HEADERS
//***********************************************

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <algorithm>
#include <map>
#include <set>
#include <eigen3/Eigen/Dense>
#include "BundleAdjustment.h"
#include "Visualizer.h"
#include <thread>
#include "PCL_visualizer.h"

class StructFromMotion{ 

  private:
  std::vector<cv::Mat>                    nImages;
  std::vector<cv::Matx34f>                nCameraPoses;
  std::vector<Features>                   nFeaturesImages;
  std::vector<Point3D>                    nReconstructionCloud;
  std::vector<Point3DRGB>                 nReconstructionCloudRGB;
  std::vector<std::string>                nImagesPath; 
  std::set<int>                           nDoneViews;
  CameraData                              cameraMatrix;
  cv::Ptr<cv::Feature2D>                  ptrFeature2D;
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

 void run_SFM( std::ifstream& file);

  bool imagesLOAD(std::ifstream& file);
void loadVisualizer();
void multithreading (std::ifstream& file);
  //===============================================
  //FEATURE DETECTION AND EXTRACTION
  //===============================================

   Features getFeatures(const cv::Mat& image);
   bool extractFeatures();

  //===============================================
  //FEATURE MATCHING
  //===============================================

  Matching getMatching(const Features& left,const Features& right);
  cv::Mat imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                        const cv::Mat& img2,const Keypoints& keypoints2,const Matching& matches);
  void imShow(const cv::Mat& matchImage,const std::string& str);



  //===============================================
  //CONVERTION KEYPOINTS TO POINTS2D
  //===============================================

  void keypoints2F(Keypoints& keypoints, Points2f& points2D);

  //===============================================
  //CAMERA MATRIX
  //===============================================

  bool getCameraMatrix();

  //===============================================
  //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
  //===============================================

  bool CheckCoherentRotation(cv::Mat& R); 

  //===============================================
  //FUNCTION ALIGNED POINTS
  //===============================================

  void AlignedPointsFromMatch(const Features& left,const Features& right,const Matching& matches,
                              Features& alignedL,Features& alignedR);

  void AlignedPoints(const Features& left,const Features& right,const Matching& matches,
                     Features& alignedL, Features& alignedR,std::vector<int>& idLeftOrigen,
                     std::vector<int>& idRightOrigen);

  //===============================================
  //FUNCTION CORRESPONDENCES 2D-3D
  //===============================================

  bool triangulateViews(const Features& left,const Features& right,const cv::Matx34f& P1,
               const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,
               const ImagePair& imagePair,std::vector<Point3D>& pointcloud);

  bool baseTriangulation();

  //===============================================
  //POINTCLOUD VISUALIZER
  //===============================================
  void updateVisualizer(const std::vector<Point3D>& pointcloud);

  //===============================================
  //INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
  //===============================================

  cv::Mat inverse(cv::Mat& matrix);

  double determinante(cv::Mat& relativeRotationCam);

  int findHomographyInliers(const Features& f1,const Features& f2,const Matching& matches);

  void addMoreViews();


  void findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,
                         const std::vector<cv::Point2f>& pts2D,cv::Matx34f& cameraPose);

  Pts3D2DPNP find2D3DMatches(ImagePair& pair);

  std::map<int,ImagePair> findBestPair();

  void mergeNewPoints(const std::vector<Point3D>& cloud);

  void adjustCurrentBundle();

  bool getCameraPose(const CameraData& intrinsics,const Matching & matches,
                     const Features& left, const Features& right, Matching& prunedMatch,
                     cv::Matx34f& Pleft, cv::Matx34f& Pright);

  void saveCloudAndCamerasToPLY(const std::string& prefix);
  void saveToPCD();
};//Fin class


