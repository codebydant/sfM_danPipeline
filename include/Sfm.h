//***********************************************
//HEADERS
//***********************************************

#include <iostream>
#include <string>
#include <map>
#include <set>
#include <eigen3/Eigen/Dense>
#include <thread>
#include <boost/filesystem.hpp>
#include <boost/multi_index/ordered_index.hpp>

#include "PCL_visualizer.h"
#include "BundleAdjustment.h"
#include "Visualizer.h"

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
  //MULTITHREADING FUNCTION
  //===============================================

  void run_SFM ();

  //===============================================
  //PIPELINE
  //===============================================

  void pipeLineSFM();

  //===============================================
  //PCL VISUALIZER
  //===============================================

  void loadVisualizer();

  //===============================================
  //IMAGES LOAD
  //===============================================

  bool imagesLOAD(const std::string&  directoryPath);

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
  //GET CAMERA MATRIX
  //===============================================

  void getCameraMatrix(const std::string str);

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

  //===============================================
  //FIND HOMOGRAPHY INLIERS
  //===============================================

  int findHomographyInliers(const Features& f1,const Features& f2,const Matching& matches);

  //===============================================
  //ADD MORE VIEWS FUNCTION
  //===============================================

  void addMoreViews();

  //===============================================
  //FIND CAMERA POSE WITH PNPRANSAC
  //===============================================


  void findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,
                         const std::vector<cv::Point2f>& pts2D,cv::Matx34f& cameraPose);

  //===============================================
  //FIND 2D-3D CORRESPONDENCES
  //===============================================

  Pts3D2DPNP find2D3DMatches(ImagePair& pair);

  //===============================================
  //FIND BEST PAIR FOR BASELINE RECONSTRUCTION
  //===============================================

  std::map<int,ImagePair> findBestPair();

  //===============================================
  //MERGE NEW POINTCLOUD
  //===============================================

  void mergeNewPoints(const std::vector<Point3D>& cloud);

  //===============================================
  //BUNDLE ADJUSTMENT ADJUSTER
  //===============================================

  void adjustCurrentBundle();

  //===============================================
  //FIND CAMERA POSE WITH ESSENTIAL MATRIX
  //===============================================

  bool getCameraPose(const CameraData& intrinsics,const Matching & matches,
                     const Features& left, const Features& right, Matching& prunedMatch,
                     cv::Matx34f& Pleft, cv::Matx34f& Pright);

  //===============================================
  //SAVE POINTCLOUD TO .PLY OR .PCD
  //===============================================

  void saveCloudAndCamerasToPLY(const std::string& prefix);
  void saveCloudToPCD();

  //===============================================
  //MESHING POINTCLOUD
  //===============================================

  void meshingPointCloud();

};//Fin class


