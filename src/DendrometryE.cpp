#include "include/DendrometryE.h"

void Dendrometry::estimate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPCL){

  std::cout << "************************************************" << std::endl;
  std::cout << "              DENDROMETRY ESTIMATION            " << std::endl;
  std::cout << "************************************************" << std::endl;

  pcl::PointXYZRGB minPt,maxPt;
  pcl::getMinMax3D(*cloudPCL,minPt,maxPt);

  cv::Point3f min=cv::Point3f(minPt.x,minPt.y,minPt.z);
  cv::Point3f max=cv::Point3f(maxPt.x,maxPt.y,maxPt.z);
  std::cout << "Max: " << max << std::endl;
  std::cout << "Min: " << min << std::endl;

  std::cout << "*** Measurements ***" << std::endl;
  std::cout << "Total Height =" << cv::norm(max-min) << std::endl;
  std::cout << "Altura copa viva=" << std::endl;
  std::cout << "Altura base de copa=" << std::endl;
  std::cout << "Altura DAP=" << 1.3 << std::endl;
  std::cout << "DAP=" << std::endl;
  std::cout << "Amplitud N-S=" << std::endl;
  std::cout << "Amplitud E-W=" << std::endl;

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

}
