//***********************************************
//HEADERS
//***********************************************

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/viz/viz3d.hpp>

//***********************************************
//ALIAS
//***********************************************

using Keypoints = std::vector<cv::KeyPoint>;
using Matching = std::vector<cv::DMatch>;
using Points2f = std::vector<cv::Point2f>;
using Points3f = std::vector<cv::Point3f>;

struct Features {

    Keypoints	kps;
    Points2f	pt2D;
    cv::Mat	descriptors;

};

struct CameraData{

  cv::Mat_<double> K;
  cv::Mat_<double> invK;
  cv::Mat_<double> distCoef;
  double fx;
  double fy;
  double cx;
  double cy;

};

struct Point3D {

    cv::Point3f pt;    
    std::map<int,int> idxImage;

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

struct MatchesforSort{

  int size;
  size_t i;
  size_t j;

};

