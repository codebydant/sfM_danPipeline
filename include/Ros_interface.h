//***********************************************
//HEADERS
//***********************************************

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


ros::Publisher pub;

 void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
 {
   // Create a container for the data.
   sensor_msgs::PointCloud2 output;

  // Do data processing here...
   output = *input;

    // Publish the data.
   pub.publish (output);
  }

