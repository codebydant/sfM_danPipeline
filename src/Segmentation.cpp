#include "include/Segmentation.h"

void Segmentation::color_based_growing_segmentation(){

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::io::loadPCDFile("MAP3D.pcd", *cloud);

  std::cout << "************************************************" << std::endl;
  std::cout << "    COLOR BASE GROWING SEGMENTATION             " << std::endl;
  std::cout << "************************************************" << std::endl;

  if(cloud->size() <= 0){
     std::cout << "Cloud reading failed. no data points found" << std::endl;
     std::exit(-1);
  }

  std::cout << "Preparing options for segmentation..." << std::endl;

  pcl::IndicesPtr indices(new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,14.0);
  pass.filter(*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(10);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  std::cout << "Input cloud:" << cloud->size() << "\n" << "Distance threshold:" << 10 << "\n"
            << "Point color threshold:" << 6 << "\n" << "Region color threshold:" << 5 << "\n"
            << "Clusters size:" << 600 << std::endl;

  std::cout << "Extracting clusters..." << std::endl;
  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);
  if(clusters.size()<=0){
       std::cerr << "Error: could not extract enough clusters." << std::endl;
       std::cout << "Extract:" << clusters.size() << " clusters. Min=600" << std::endl;
       std::exit(-1);
  }
  std::cout << "Extract:" << clusters.size() << " clusters" << std::endl;

  std::cout << "Getting color cloud..." << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  std::cout << "************************************************" << std::endl;
  std::cout << "************************************************" << std::endl;

  std::cout << "Showing 3D Mapping segmented..." << std::endl;

  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("MAP3D Segmented",true);
  viewer.addPointCloud(colored_cloud);
  std::cout << "Press q to finish segmentation proccess and start dendrometry estimation..." << std::endl;
  while(!viewer.wasStopped ()){
       viewer.spin();
  }

}
