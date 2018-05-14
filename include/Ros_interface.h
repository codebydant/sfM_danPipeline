//***********************************************
//HEADERS
//***********************************************
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include "Utilities.h"

class ROSInterface{

  private:
     pcl::PointCloud<pcl::PointXYZRGB> cloudPCL;
     ros::Publisher pub;    

  public:


     ROSInterface(){

       // Create a ROS publisher for the output point cloud
       ros::NodeHandle nh;
       std::string tf_frame;
       tf_frame="/base_link";
       pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("/output", 1,false);
       nh.param("frame_id", tf_frame, std::string("/base_link"));
       std::cout << "Initializing ROS interface..." << std::endl;

     }

    ~ROSInterface(){}

     bool fromPoint3DToPCL(const std::vector<Point3D>& inputPCL_Cloud,
                              const std::vector<cv::Vec3b>& inputPCL_CloudRGB);
     void addPointCloudToRVIZ(const std::vector<Point3D>& inputPCL_Cloud,
                              const std::vector<cv::Vec3b>& inputPCL_CloudRGB);

};
