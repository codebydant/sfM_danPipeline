#include "include/Visualizer.h"

void Visualizer::addPointCloudtoPCL(const std::vector<Point3D>& inputPointCloud){

  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("3D Reconstruction",true);

  viewer.setPosition(0,0);
  viewer.setSize(640,480);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setCameraPosition(2,2,11,0,0,0,0);
  viewer.resetCamera();
  while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed


      viewer.removeAllPointClouds();
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ> ());
      // Fill in the cloud data
      cloudPCL->width    = inputPointCloud.size();
      cloudPCL->height   = 1;
      cloudPCL->is_dense = false;
      cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

      for (size_t i = 0; i < cloudPCL->points.size (); ++i){
         Point3D pt3d = inputPointCloud[i];
         cloudPCL->points[i].x = pt3d.pt.x;
         cloudPCL->points[i].y = pt3d.pt.y;
         cloudPCL->points[i].z = pt3d.pt.z;
      }
      // Define R,G,B colors for the point cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 0, 255, 0);
      //We add the point cloud to the viewer and pass the color handler

      viewer.addPointCloud (cloudPCL, cloud_color, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

      viewer.spinOnce(100);
  }

}


