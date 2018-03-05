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



struct Pt2DAligned{

  Points2f Pt2D_left;
  Points2f Pt2D_right;
  std::vector<int> trainIdx;
  std::vector<int> queryIdx;

};

struct idImagePair{

  size_t   i;
  size_t   j;
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
    std::map<int,int> pt3D2D;

};

struct ImagePair{
  size_t left;
  size_t right;

};

struct Point3D2DMatch{

  Points2f pts2D;
  Points3f pts3D;

};


void AlignedPointsFromMatch(Features& left,Features& right, Matching& matches,
                            Features& featuresLeftAligned,Features& featuresRightAligned);

