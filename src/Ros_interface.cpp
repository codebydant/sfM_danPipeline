#include "include/Ros_interface.h"

void ROSInterface::addPointCloudToRVIZ(const std::vector<Point3D> &inputPCL_Cloud,
                           const std::vector<cv::Vec3b>& inputPCL_CloudRGB){

    bool success = fromPoint3DToPCL(inputPCL_Cloud,inputPCL_CloudRGB);
    if(not success){

      std::cout << "Could not convert pointcloud to PCL pointcloud data type"
                << std::endl;
    }

    ros::Rate loop_rate(10);
    /*
    pcl::PointCloud<pcl::PointXYZRGB> outCloudTransform;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0,0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world","base_link"));

    pcl_ros::transformPointCloud(cloudPCL,outCloudTransform,transform);
    //outCloudTransform.header.stamp = ros::Time::now().toNSec();
    //pcl_conversions::toPCL(ros::Time::now(), outCloudTransform.header.stamp);
*/
    pub.publish(cloudPCL);
    ros::spinOnce ();
    loop_rate.sleep ();
}

bool ROSInterface::fromPoint3DToPCL(const std::vector<Point3D> &inputPCL_Cloud,
                                    const std::vector<cv::Vec3b>& inputPCL_CloudRGB){

  for(size_t i = 0; i < inputPCL_Cloud.size(); ++i){
      Point3D pt3d = inputPCL_Cloud[i];
      cv::Vec3b rgbv(255,255,255);
      pcl::PointXYZRGB pclp;
      pclp.x  = pt3d.pt.x;
      pclp.y  = pt3d.pt.y;
      pclp.z  = pt3d.pt.z;
      rgbv = inputPCL_CloudRGB[i];

      // RGB color, needs to be represented as an integer
      uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
      pclp.rgb = *reinterpret_cast<float*>(&rgb);
      cloudPCL.points.push_back(pclp);
   }

   cloudPCL.width = (uint32_t) cloudPCL.points.size(); // number of points
   cloudPCL.height = 1;	// a list, one row of data
   cloudPCL.header.frame_id ="/base_link";


   if(cloudPCL.points.size()<=0){
     return false;
   }else{
     return true;
   }

}
