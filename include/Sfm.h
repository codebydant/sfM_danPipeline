//***********************************************
//HEADERS
//***********************************************
#include <iostream>
#include <string>
#include <set>
#include <eigen3/Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/thread/thread.hpp>
#include "DendrometryE.h"
#include <chrono>


class StructFromMotion{ 

  private:
    std::vector<cv::Mat>                    mColorImages;
    std::vector<cv::Mat>                    mGrayImages;
    std::vector<cv::Mat>                    nImages;
    std::vector<cv::Matx34d>                nCameraPoses;
    std::vector<std::string>                nImagesPath;
    std::string                             pathImages;
    std::set<int>                           nDoneViews;
    std::set<int>                           nGoodViews;
    Intrinsics                              cameraMatrix;
    float                                   NN_MATCH_RATIO;
    std::vector<std::vector<cv::KeyPoint>>  imagesKeypoints;
    std::vector<cv::Mat>                    imagesDescriptors;
    std::vector<std::vector<cv::Point2d>>   imagesPts2D;
    int                                     detector;

  public:
    std::vector<Point3D>                    nReconstructionCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr     cloudPCL;

    //===============================================
    //CONSTRUCTOR
    //===============================================
    StructFromMotion(){

      /* @ SIFT = 1,@ AKAZE = 2,@ ORB = 3 */
      int selector = 1;
      switch(selector){

        case 1:
           detector = 1;
          break;
        case 2:
           detector = 2;
          break;
        case 3:
           detector = 3;
          break;
        default:
          break;
      }

      //less matches <-- NN_MATCH_RATIO --> more matches
      NN_MATCH_RATIO = 0.8f;
    }
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    bool map3D();
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
    void getFeature(const cv::Mat& image,const int& numImage);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    void extractFeature();
    //===============================================
    //FEATURE MATCHING
    //===============================================
    void getMatching(const int& queryImage,const int& trainImage,Matching* goodMatches);
    void prunedMatchingWithHomography(const int& idx_query, const int& idx_train,
                                                        const Matching& goodMatches,Matching* prunedMatch);
    //===============================================
    //CONVERTION KEYPOINTS TO POINTS2D
    //===============================================
    void keypointstoPoints(Keypoints& keypoints, Points2d& points2D);
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
    void AlignedPointsFromMatch(const Points2d& queryImg,const Points2d& trainImg,const Matching& matches,
                                Points2d& alignedL,Points2d& alignedR);
    void AlignedPoints(const Points2d& queryImg,const Points2d& trainImg,const Matching& matches,
                       Points2d& alignedL, Points2d& alignedR,std::vector<int>& idLeftOrigen,
                       std::vector<int>& idRightOrigen);
    //===============================================
    //FUNCTION CORRESPONDENCES 2D-3D
    //===============================================
    bool triangulateViews(const Points2d& left,const Points2d& right,const cv::Matx34d& P1,
                 const cv::Matx34d& P2,const Matching& matches,const Intrinsics& matrixK,
                 const std::pair<int,int>& imagePair,std::vector<Point3D>& pointcloud);
    //===============================================
    //MULTITHREADING FUNCTION
    //===============================================
    bool baseReconstruction();
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
    int findHomographyInliers(const int& f1,const int& f2,const Matching& matches);
    //===============================================
    //ADD MORE VIEWS FUNCTION
    //===============================================
    bool addMoreViews();
    //===============================================
    //FIND CAMERA POSE WITH PNPRANSAC
    //===============================================
    bool findCameraPosePNP(const Intrinsics& matrixK,const std::vector<cv::Point3d>& pts3D,
                           const std::vector<cv::Point2d>& pts2D,cv::Matx34d& cameraPose);
    //===============================================
    //FIND 2D-3D CORRESPONDENCES
    //===============================================
    void find2D3DMatches(const int& NEW_VIEW,
                                           std::vector<cv::Point3d>& points3D,
                                           std::vector<cv::Point2d>& points2D,Matching& bestMatches,int& DONEVIEW);
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
    bool getCameraPose(const Intrinsics& intrinsics,const int& idx_query,const int& idx_train,const Matching & matches,
                       const Points2d& left, const Points2d& right,
                       cv::Matx34d& Pleft, cv::Matx34d& Pright);
    //===============================================
    //MESHING POINTCLOUD
    //===============================================
    void meshingPointCloud();

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

    void MatchFeatures(int idx_i, int idx_j, std::vector<cv::DMatch>* matches);

    void keypointstoPoints2F(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::Point2f>& points2D);

};


