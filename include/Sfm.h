#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/viz/vizcore.hpp"
#include "opencv2/viz/viz3d.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>


class StructFromMotion{

  cv::Mat_<cv::Vec3b> image1, image2;

public:

StructFromMotion(){}

StructFromMotion(cv::Mat& img1,cv::Mat& img2);

std::vector<cv::Point3d> recon();

void setConstructor(cv::Mat& img1,cv::Mat& img2);

void cameraPoseAcumulada();

cv::Mat imageMatching(cv::Mat& img1,std::vector<cv::KeyPoint>& keypoints1,cv::Mat& img2, std::vector<cv::KeyPoint>& keypoints2,std::vector<cv::DMatch>& matches);

void matchingImShow(cv::Mat& matchImage);

std::vector<cv::Point2f> keypoints2F(std::vector<cv::KeyPoint>& keypoints,std::vector<cv::DMatch>& matches);

std::vector<cv::KeyPoint> obtenerKeypoints (cv::Mat& image);

std::vector<cv::DMatch> obtenerMatches(cv::Mat& img1,cv::Mat& img2,std::vector<cv::KeyPoint>& keypoints1,std::vector<cv::KeyPoint>& keypoints2);

cv::Mat_<double> getCameraMatrix();

cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1);

cv::Mat_<double> LinearLSTriangulation4(cv::Point2d u,cv::Matx34d P,cv::Point2d u1,cv::Matx34d P1);

cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,
                                            cv::Matx34d P,          //camera 1 matrix
                                            cv::Point3d u1,
                                            cv::Matx34d P1          //camera 2 matrix
                                            ) ;

cv::Mat_<double> findEssentialMatrix(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,cv::Mat_<double>& cameraMatrix);

cv::Mat inverse(cv::Mat& matrix);

void cameraPose(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,double& fx,double cx,double cy,cv::Mat& rot,cv::Mat& tra,cv::Mat& inliers,cv::Mat_<double>& essentialMatrix );

void projection(const cv::Mat& relativeRotationCam,const cv::Mat& relativeTranslaCam, cv::Mat_<double>& projection1, cv::Mat_<double>& projection2);

std::vector<cv::Point3d> triangulation(std::vector<cv::Point2f>& points1,std::vector<cv::Point2f>& points2,cv::Mat_<double>& projection1,cv::Mat_<double>& projection2,cv::Mat& inliers);

void visualizerPointCloud(cv::Matx33d& cameraMatrix,cv::Mat& img1,cv::Mat& img2,cv::Mat& cameraR,cv::Mat& cameraT,std::vector<cv::Point3d>& pointcloud);

void recon( std::ifstream& file);

void cargarFrame( std::ifstream& file,cv::Mat& image1,cv::Mat& image2);



};//Fin class


