//***********************************************
//HEADERS
//***********************************************
#include <iostream>
#include <string>
#include <thread>
#include <set>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>
#include "DendrometryE.h"


class StructFromMotion{ 

  private:
    std::vector<cv::Mat>                    nImages;
    std::vector<cv::Matx34f>                nCameraPoses;
    std::vector<Feature>                    nFeatureImages;
    std::vector<std::string>                nImagesPath;
    std::string                             pathImages;
    std::set<int>                           nDoneViews;
    std::set<int>                           nGoodViews;
    CameraData                              cameraMatrix;
    cv::Ptr<cv::Feature2D>                  ptrFeature2D;
    cv::Ptr<cv::DescriptorMatcher>          matcher;
    float                                   NN_MATCH_RATIO;
    cv::Ptr<cv::Feature2D> detector;

  public:
    std::vector<Point3D>                    nReconstructionCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     cloudPCL;

    //===============================================
    //CONSTRUCTOR
    //===============================================
    StructFromMotion(){
      /* @ FLANNBASED = 1,@ BRUTEFORCE = 2,@ BRUTEFORCE_L1 = 3,@ BRUTEFORCE_HAMMING = 4,
         @ BRUTEFORCE_HAMMINGLUT = 5,@ BRUTEFORCE_SL2 = 6  */
      int selector = 1;
      switch(selector){

        case 1:
           detector= cv::xfeatures2d::SIFT::create();
          break;
        case 2:
           detector= cv::AKAZE::create();
          break;
        case 3:
           detector= cv::ORB::create(10000,1.2,8,15,0,2,0,31);
          break;
        default:
          break;

      }
      matcher = cv::DescriptorMatcher::create(2);
      NN_MATCH_RATIO = 0.8f;
    }
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    bool run_SFM ();   
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
    Feature getFeature(const cv::Mat& image);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    bool extractFeature();
    //===============================================
    //FEATURE MATCHING
    //===============================================
    Matching getMatching(const Feature& queryImage,const Feature& trainImage);
    //===============================================
    //CONVERTION KEYPOINTS TO POINTS2D
    //===============================================
    void keypoints2F(Keypoints& keypoints, Points2f& points2D);
    //===============================================
    //GET CAMERA MATRIX
    //===============================================
    bool getCameraMatrix(const std::string str);
    //===============================================
    //FUNCTION CHECK ROTATION MATRIX (Must be det=1)
    //===============================================
    bool CheckCoherentRotation(cv::Mat& R);
    //===============================================
    //FUNCTION ALIGNED POINTS
    //===============================================
    void AlignedPointsFromMatch(const Feature& queryImg,const Feature& trainImg,const Matching& matches,
                                Feature& alignedL,Feature& alignedR);
    void AlignedPoints(const Feature& queryImg,const Feature& trainImg,const Matching& matches,
                       Feature& alignedL, Feature& alignedR,std::vector<int>& idLeftOrigen,
                       std::vector<int>& idRightOrigen);
    //===============================================
    //FUNCTION CORRESPONDENCES 2D-3D
    //===============================================
    bool triangulateViews(const Feature& left,const Feature& right,const cv::Matx34f& P1,
                 const cv::Matx34f& P2,const Matching& matches,const CameraData& matrixK,
                 const std::pair<int,int>& imagePair,std::vector<Point3D>& pointcloud);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    bool baseTriangulation();
    //===============================================
    //POINTCLOUD VISUALIZER
    //===============================================
    void updateVisualizer(const std::vector<Point3D>& pointcloud);
    //===============================================
    //INVERSE MATRIX-DETERMINANT FUNCTION EIGEN
    //===============================================
    cv::Mat inverse(cv::Mat& matrix);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    double determinante(cv::Mat& relativeRotationCam);
    //===============================================
    //FIND HOMOGRAPHY INLIERS
    //===============================================
    int findHomographyInliers(const Feature& f1,const Feature& f2,const Matching& matches);
    //===============================================
    //ADD MORE VIEWS FUNCTION
    //===============================================
    bool addMoreViews();
    //===============================================
    //FIND CAMERA POSE WITH PNPRANSAC
    //===============================================
    bool findCameraPosePNP(const CameraData& matrixK,const std::vector<cv::Point3f>& pts3D,
                           const std::vector<cv::Point2f>& pts2D,cv::Matx34f& cameraPose);
    //===============================================
    //FIND 2D-3D CORRESPONDENCES
    //===============================================
    void find2D3DMatches(const int& NEW_VIEW,
                                           std::vector<cv::Point3f>& points3D,
                                           std::vector<cv::Point2f>& points2D,Matching& bestMatches,int& DONEVIEW);
    //===============================================
    //FIND BEST PAIR FOR BASELINE RECONSTRUCTION
    //===============================================
    std::map<float,std::pair<int,int>>  findBestPair();
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
                       const Feature& left, const Feature& right,
                       cv::Matx34f& Pleft, cv::Matx34f& Pright);
    //===============================================
    //MESHING POINTCLOUD
    //===============================================
    void meshingPointCloud();
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    Matching matchingFor2D3D(Feature& feature1,Feature& feature2);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    void createPointsTxt();
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    void PMVS2();
    void fromPoint3DToPCLCloud(const std::vector<Point3D> &input_cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudPCL);
    void cloudPointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud);
    void removePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &filterCloud);
    void create_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PolygonMesh &mesh);
    void vizualizeMesh(pcl::PolygonMesh &mesh);

    void optical_flow_feature_match();
    void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches);

};


