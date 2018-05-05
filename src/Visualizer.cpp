#include "include/Visualizer.h"

void Visualizer::addPointCloudtoPCL(const std::vector<Point3D>& inputPointCloud,const std::vector<cv::Vec3b>& cloudRGB){

  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("3D Reconstruction",true);

  viewer.setPosition(0,0);
  viewer.setSize(640,480);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setCameraPosition(2,2,11,0,0,0,0);
  viewer.resetCamera();
  while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed

      viewer.removeAllPointClouds();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZRGB> ());
      for (unsigned int i = 0; i < inputPointCloud.size(); ++i){
         Point3D pt3d = inputPointCloud[i];
         cv::Vec3b rgbv(255,255,255);
         pcl::PointXYZRGB pclp;
         pclp.x  = pt3d.pt.x;
         pclp.y  = pt3d.pt.y;
         pclp.z  = pt3d.pt.z;
         rgbv = cloudRGB[i];

         // RGB color, needs to be represented as an integer
         uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
         pclp.rgb = *reinterpret_cast<float*>(&rgb);
        cloudPCL->push_back(pclp);
      }

      cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
      cloudPCL->height = 1;	// a list, one row of data

      viewer.addPointCloud (cloudPCL, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

      viewer.spinOnce(100);
  }

}


