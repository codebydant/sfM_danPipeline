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

     void addPointCloudtoPCL(const std::vector<Point3D>& inputPointCloud,
                             const std::vector<cv::Vec3b>& cloudRGB);

};

