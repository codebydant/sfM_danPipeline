#include "include/Visualizer.h"

void showPCLVisualizer(){
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MAP3D"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
   // viewer->initCameraParameters ();
    viewer->setCameraPosition(10,10,3,0,0,0,0);
    viewer->spinOnce(100);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
      }
}

boost::thread* _t = NULL;
void Visualizer::RunVisualizationThread() {
        _t = new boost::thread(showPCLVisualizer);
}

void Visualizer::updatePointCloud(){
  /*
  viewer->removeAllPointClouds();
  viewer->updatePointCloud(PointCloudPCL,"original_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                           2, "original_cloud");
  viewer->spinOnce(100);
*/
}

void Visualizer::addPointCloudToPCL(const std::vector<Point3D>& inputCloud,
                                    const std::vector<cv::Vec3b>& inputCloudRGB){
/*
  fromPoint3DToPCL(inputCloud,inputCloudRGB);
  viewer->removeAllPointClouds();
  viewer->addPointCloud(PointCloudPCL, "original_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                           2, "original_cloud");
  updatePointCloud();
*/
}

void Visualizer::fromPoint3DToPCL(const std::vector<Point3D> &inputPCL_Cloud,
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
      PointCloudPCL->points.push_back(pclp);
   }

   PointCloudPCL->width = (uint32_t) PointCloudPCL->points.size(); // number of points
   PointCloudPCL->height = 1;	// a list, one row of data
   PointCloudPCL->header.frame_id ="framePointCloud";
   PointCloudPCL->is_dense = false;
}
