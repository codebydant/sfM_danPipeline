//***********************************************
//HEADERS
//***********************************************

#include <thread>
#include <iostream>
#include <string>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <Eigen/Dense>
#include "BundleAdjustment.h"

class StructFromMotion : public Visualizer{

    private:
    std::vector<cv::Mat>                    nImages;
    std::vector<cv::Matx34f>                nCameraPoses;
    std::vector<Feature>                    nFeatureImages;
    std::vector<Point3DRGB>                 nReconstructionCloudRGB;

    std::vector<std::string>                nImagesPath;
    std::set<int>                           nDoneViews;
    std::set<int>                           nGoodViews;
    CameraData                              cameraMatrix;
    cv::Ptr<cv::Feature2D>                  ptrFeature2D;
    cv::Ptr<cv::DescriptorMatcher>          matcherFlan;
    double                                  NN_MATCH_RATIO;


    public:
    std::vector<Point3D>                    nReconstructionCloud;
 Visualizer    interface;


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

      ptrFeature2D = cv::ORB::create(2000.0);
      matcherFlan = cv::DescriptorMatcher::create("BruteForce-Hamming");
      NN_MATCH_RATIO = 0.8f;

    }

    ~StructFromMotion(){

     }

    //===============================================
    //CARGA DE IMAGENES
    //===============================================

    void updatePointCloud();



   void pipeLine();

    bool imagesLOAD(const std::string& directoryPath);
  void loadVisualizer();
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

    bool getCameraMatrix(const std::string& str);

    //===============================================
    //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
    //===============================================

    bool CheckCoherentRotation(cv::Mat& R);

    //===============================================
    //FUNCTION ALIGNED POINTS
    //===============================================

    void AlignedPointsFromMatch(const Feature& left,const Feature& right,const Matching& matches,
                                Feature& alignedL,Feature& alignedR);

    void AlignedPoints(const Feature& left,const Feature& right,const Matching& matches,
                       Feature& alignedL, Feature& alignedR,std::vector<int>& idLeftOrigen,
                       std::vector<int>& idRightOrigen);

    //===============================================
    //FUNCTION CORRESPONDENCES 2D-3D
    //===============================================

    bool triangulateViews(const Feature& left,const Feature& right,const cv::Matx34f& P1,
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

    int findHomographyInliers(const Feature& f1,const Feature& f2,const Matching& matches);

    void addMoreViews();

    Matching matchingFor2D3D(Feature& feature1,Feature& feature2);


    bool findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,
                           const std::vector<cv::Point2f>& pts2D,cv::Matx34f& cameraPose);

    std::map<int,Image3D2DMatch> find2D3DMatches();

    std::map<int,ImagePair> findBestPair();

    void mergeNewPoints(const std::vector<Point3D>& cloud);

    void adjustCurrentBundle();

    bool getCameraPose(const CameraData& intrinsics,const Matching & matches,
                       const Feature& left, const Feature& right, Matching& prunedMatch,
                       cv::Matx34f& Pleft, cv::Matx34f& Pright);

    void saveCloudAndCamerasToPLY(const std::string& prefix);
    void saveToPCD();
  };//Fin class



