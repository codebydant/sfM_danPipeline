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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPCL;


  public:

     Visualizer(){

     }

     ~Visualizer(){

     }

     void addPointCloudToPCL(const std::vector<Point3D>& inputCloud,
                             const std::vector<cv::Vec3b>& inputCloudRGB);
     void fromPoint3DToPCL(const std::vector<Point3D> &inputPCL_Cloud,
                                       const std::vector<cv::Vec3b>& inputPCL_CloudRGB);
     void updatePointCloud();
     void RunVisualizationThread();

};

