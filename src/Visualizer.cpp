#include "include/Visualizer.h"

void Visualizer::showPCLVisualizer(){


  /*
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
      */
}

//boost::thread* _t = NULL;
void Visualizer::RunVisualizationThread() {
        //_t = new boost::thread(showPCLVisualizer);
}

void Visualizer::updatePointCloud(){



}

void Visualizer::addPointCloudToPCL(const std::vector<Point3D>& inputCloud,
                                    const std::vector<cv::Vec3b>& inputCloudRGB){

  pcl::visualization::PCLVisualizer viewer = pcl::visualization::PCLVisualizer("MAP3D",true);
  viewer.setPosition(0,0);
  viewer.setSize(800,600);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer.setCameraPosition(12,14,10,0,0,0,0);
  viewer.resetCamera();

  while(!viewer.wasStopped()){

          viewer.addCoordinateSystem (1.0, "ucs", 0);
          viewer.removeAllPointClouds();
          fromPoint3DToPCL(inputCloud,inputCloudRGB);
          viewer.addPointCloud(cloudPCL, "original_cloud");
          viewer.spin();

      }

}

void Visualizer::fromPoint3DToPCL(const std::vector<Point3D> &inputPCL_Cloud,
                                  const std::vector<cv::Vec3b>& inputPCL_CloudRGB){

  cloudPCL.reset(new pcl::PointCloud<pcl::PointXYZRGB> ());

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
      cloudPCL->push_back(pclp);
   }

   cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
   cloudPCL->height = 1;	// a list, one row of data
   cloudPCL->header.frame_id ="map3d";
   cloudPCL->is_dense = false;


}
