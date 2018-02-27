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
using MatchesVector = std::vector<cv::DMatch>;
using Points2d = std::vector<cv::Point2f>;
using Points2f = std::vector<cv::Point2f>;
using Points3f = std::vector<cv::Point3f>;
using Points3d = std::vector<cv::Point3d>;

struct Features {

    Keypoints	kps;
    Points2d	pt2D;
    cv::Mat	descriptors;
    std::string imagePath;

  };

struct Matches {

    MatchesVector matches12;
    MatchesVector matches21;
    MatchesVector goodMatches;
    std::vector<int> trainIdx;
    std::vector<int> queryIdx;
    Features leftReference;
    Features rightReference;


  };

struct PointCloud {

    Points3d points3D;
  };


void AlignedPointsFromMatch(Features& left,Features& right, MatchesVector& matches,
                            Features& featuresLeftAligned,Features& featuresRightAligned);

