//***********************************************
//HEADERS
//***********************************************
#include <iostream>
#include <string>
#include <thread>
#include <set>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include "BundleAdjustment.h"

class StructFromMotion {

    private:

      std::vector<cv::Mat>                    nImages;
      std::vector<cv::Matx34f>                nCameraPoses;
      std::vector<Feature>                    nFeatureImages;
      std::vector<Point3DRGB>                 nReconstructionCloudRGB;
      std::vector<Point3D>                    nReconstructionCloud;
      std::vector<std::string>                nImagesPath;
      std::set<int>                           nDoneViews;
      std::set<int>                           nGoodViews;
      CameraData                              cameraMatrix;
      cv::Ptr<cv::Feature2D>                  ptrFeature2D;
      cv::Ptr<cv::DescriptorMatcher>          matcherFlan;
      double                                  NN_MATCH_RATIO;

   public:

   //===============================================
   //CONSTRUCTOR
   //===============================================

      StructFromMotion(){

        /* @ FLANNBASED = 1,
           @ BRUTEFORCE = 2,
           @ BRUTEFORCE_L1 = 3,
           @ BRUTEFORCE_HAMMING = 4,
           @ BRUTEFORCE_HAMMINGLUT = 5,
           @ BRUTEFORCE_SL2 = 6   */

       ptrFeature2D = cv::ORB::create(5000.0);
       matcherFlan = cv::DescriptorMatcher::create("BruteForce-Hamming");
       NN_MATCH_RATIO = 0.9f;
    }

   //===============================================
   //DESTRUCTOR
   //===============================================
    ~StructFromMotion(){}
   //===============================================
   //PIPELINE
   //===============================================
   void pipeLine();      
   //===============================================
   //IMAGES LOAD
   //===============================================
   bool imagesLOAD(const std::string& directoryPath);
   //===============================================
   //MULTITHREADING FUNCTION
   //===============================================
   void run_SFM();
   //===============================================
   //FEATURE DETECTION AND EXTRACTION
   //===============================================
   Feature getFeature(const cv::Mat& image);
   bool extractFeature();
   //===============================================
   //FEATURE MATCHING
   //===============================================
   Matching getMatching(const Feature& left,const Feature& right);
   //===============================================
   //DRAW MATCHES
   //===============================================
   cv::Mat imageMatching(const cv::Mat& img1,const Keypoints& keypoints1,
                          const cv::Mat& img2,const Keypoints& keypoints2,const Matching& matches);
   //===============================================
   //IMAGE PRINT
   //===============================================
   void imShow(const cv::Mat& matchImage,const std::string& str);
   //===============================================
   //GET CAMERA MATRIX
   //===============================================
   bool getCameraMatrix(const std::string& str);
   //===============================================
   //CHECK ROTATION MATRIX (Must be det=1)
   //===============================================
   bool CheckCoherentRotation(cv::Mat& R);
   //===============================================
   //ALIGNED POINTS
   //===============================================
    void AlignedPointsFromMatch(const Feature& left,const Feature& right,const Matching& matches,
                                Feature& alignedL,Feature& alignedR);
    void AlignedPoints(const Feature& left,const Feature& right,const Matching& matches,
                       Feature& alignedL, Feature& alignedR,std::vector<int>& idLeftOrigen,
                       std::vector<int>& idRightOrigen);
    //===============================================
    //TRIANGULATE VIEWS
    //===============================================
    bool triangulateViews(const Feature& left,const Feature& right,const cv::Matx34f& P1,
                 const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,
                 const ImagePair& imagePair,std::vector<Point3D>& pointcloud);    
    //===============================================
    //BASE POINTCLOUD
    //===============================================
    bool baseTriangulation();
    //===============================================
    //INVERSE MATRIX-EIGEN
    //===============================================
    cv::Mat inverse(cv::Mat& matrix);
    //===============================================
    //DETERMINANTE-EIGEN
    //===============================================
    double determinante(cv::Mat& relativeRotationCam);
    //===============================================
    //HOMOGRAPHY INLIERS
    //===============================================
    int findHomographyInliers(const Feature& f1,const Feature& f2,const Matching& matches);    
    //===============================================
    //ADD MORE VIEWS
    //===============================================
    void addMoreViews();    
    //===============================================
    //2D3D CORRESPONDENCES
    //===============================================
    Matching matchingFor2D3D(Feature& feature1,Feature& feature2);    
    //===============================================
    //CAMERA POSE - PNP RANSAC
    //===============================================
    bool findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,
                           const std::vector<cv::Point2f>& pts2D,cv::Matx34f& cameraPose);
    //===============================================
    //CAMERA POSE - ESSENTIAL
    //===============================================
    bool getCameraPose(const CameraData& intrinsics,const Matching & matches,
                       const Feature& left, const Feature& right, Matching& prunedMatch,
                       cv::Matx34f& Pleft, cv::Matx34f& Pright);
    //===============================================
    //2D3D CORRESPONDENCES
    //===============================================
    std::map<int,Image3D2DMatch> find2D3DMatches();    
    //===============================================
    //BEST IMAGE PAIR
    //===============================================
    std::map<int,ImagePair> findBestPair();    
    //===============================================
    //MERGE NEW POINTCLOUD
    //===============================================
    void mergeNewPoints(const std::vector<Point3D>& cloud);    
    //===============================================
    //BUNDLE ADJUSTER
    //===============================================
    void adjustCurrentBundle();
    //===============================================
    //SAVE POINTCLOUD
    //===============================================
    void saveCloudAndCamerasToPLY(const std::string& prefix);
    void saveToPCD();

    void keypoints2F(Keypoints& keypoints, Points2f& points2D);

};//Fin class



