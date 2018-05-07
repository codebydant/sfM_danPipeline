//***********************************************
//HEADERS
//***********************************************
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Utilities.h"

class Visualizer{

  private:

  public:

     Visualizer(){

     }

     ~Visualizer(){

     }

     void showPCLVisualizer();
     void addPointCloudToPCL(const std::vector<Point3D>& inputCloud,
                             const std::vector<cv::Vec3b>& inputCloudRGB);
      void addTrajectory(std::vector<cv::Matx34f>& path);

     std::vector<Point3D> PointCloudPCL;
     std::vector<cv::Vec3b> PointCloudPCLRGB;
     /*
     inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb);
     void visualizerShowCamera(Eigen::MatrixX3f& R, Eigen::Vector3f& _t, float r, float g, float b) ;
     inline std::vector<Eigen::Matrix<float,6,1> > AsVector(const Eigen::Matrix<float,6,1>& p1,
                                                            const Eigen::Matrix<float,6,1>& p2);

     inline Eigen::Matrix<float,6,1> Eigen2Eigen(Eigen::Vector3f v, Eigen::Vector3f rgb);


     void visualizerShowCamera(cv::Matx33f& R, cv::Vec3f& t, float r, float g, float b) ;





     std::vector<cv::Matx34d> trajectory;
     */
};

