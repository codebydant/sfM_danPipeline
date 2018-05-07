//***********************************************
//HEADERS
//***********************************************

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>

//***********************************************
//ALIAS
//***********************************************

using Keypoints = std::vector<cv::KeyPoint>;
using Matching = std::vector<cv::DMatch>;
using Points2f = std::vector<cv::Point2f>;
using Points3f = std::vector<cv::Point3f>;

struct Feature {

    Keypoints	kps;
    Points2f	pt2D;
    cv::Mat	descriptors;

};

struct CameraData{

  cv::Mat_<float> K;
  cv::Mat invK;
  cv::Mat distCoef;
  cv::Matx33f K3x3;
  std::vector<double> distCoefVec;
  float fx;
  float fy;
  float cx;
  float cy;

};

struct Point3D {

    cv::Point3f pt;    
    std::map<int,int> idxImage;
    size_t id;

};

struct Point3DRGB{

    Point3D pt;
    cv::Scalar rgb;
};

struct ImagePair{
  size_t left;
  size_t right;

};

struct Pts3D2DPNP{

  Points2f pts2D;
  Points3f pts3D;

};

struct Image3D2DMatch{

  Pts3D2DPNP pts;
  size_t     left;
  size_t     right;
};


