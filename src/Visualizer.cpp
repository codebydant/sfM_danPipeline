#include "include/Visualizer.h"

void Visualizer::showPCLVisualizer(){

  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("3D Reconstruction",true);

  viewer.setPosition(0,0);
  viewer.setSize(800,600);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setCameraPosition(2,2,11,0,0,0);
  viewer.resetCamera();

  while(!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed

      viewer.removeAllPointClouds();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZRGB> ());
      for (unsigned int i = 0; i < PointCloudPCL.size(); ++i){
         Point3D pt3d = PointCloudPCL[i];
         cv::Vec3b rgbv(255,255,255);
         pcl::PointXYZRGB pclp;
         pclp.x  = pt3d.pt.x;
         pclp.y  = pt3d.pt.y;
         pclp.z  = pt3d.pt.z;
         rgbv = PointCloudPCLRGB[i];

         // RGB color, needs to be represented as an integer
         uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 | (uint32_t)rgbv[0]);
         pclp.rgb = *reinterpret_cast<float*>(&rgb);
        cloudPCL->push_back(pclp);
      }

      cloudPCL->width = (uint32_t) cloudPCL->points.size(); // number of points
      cloudPCL->height = 1;	// a list, one row of data

      viewer.addPointCloud (cloudPCL, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
/*
      std::vector<cv::Matx34d>& v=trajectory;

      for (unsigned int i = 0; i < v.size(); i++) {

           cv::Matx33f R;
           R(0, 0) = v[i](0, 0); R(0, 1) = v[i](0, 1); R(0, 2) = v[i](0, 2);
           R(1, 0) = v[i](1, 0); R(1, 1) = v[i](1, 1); R(1, 2) = v[i](1, 2);
           R(2, 0) = v[i](2, 0); R(2, 1) = v[i](2, 1); R(2, 2) = v[i](2, 2);
           visualizerShowCamera(R, cv::Vec3f(v[i](0, 3), v[i](1, 3), v[i](2, 3)), 255,0,0);
      }
      */
      viewer.spinOnce(100);
  }
}

void Visualizer::addTrajectory(std::vector<cv::Matx34f>& path){
  //trajectory = path;
}

void Visualizer::addPointCloudToPCL(const std::vector<Point3D>& inputCloud,
                                    const std::vector<cv::Vec3b>& inputCloudRGB){
  PointCloudPCL = inputCloud;
  PointCloudPCLRGB = inputCloudRGB;
}
/*
inline pcl::PointXYZRGB Eigen2PointXYZRGB(Eigen::Vector3f v, Eigen::Vector3f rgb) {
  pcl::PointXYZRGB p(rgb[0],rgb[1],rgb[2]);
  p.x = v[0]; p.y = v[1]; p.z = v[2];
  return p;
}


inline std::vector<Eigen::Matrix<float,6,1> > AsVector(const Eigen::Matrix<float,6,1>& p1,
                                                       const Eigen::Matrix<float,6,1>& p2) {
  std::vector<Eigen::Matrix<float,6,1> > v(2);
  v[0] = p1; v[1] = p2; return v;

}

inline Eigen::Matrix<float,6,1> Eigen2Eigen(Eigen::Vector3f v, Eigen::Vector3f rgb) {
  return (Eigen::Matrix<float,6,1>() << v[0],v[1],v[2],rgb[0],rgb[1],rgb[2]).finished();
}


void visualizerShowCamera2(Eigen::MatrixX3f& R,Eigen::Vector3f& _t, float r, float g, float b) {

double s=0.01;
	Eigen::Vector3f  t = -R.transpose() * _t;

	Eigen::Vector3f   vright = R.row(0).normalized() * s;
	Eigen::Vector3f   vup = -R.row(1).normalized() * s;
	Eigen::Vector3f   vforward = R.row(2).normalized() * s;

	Eigen::Vector3f  rgb(r,g,b);

	pcl::PointCloud<pcl::PointXYZRGB> mesh_cld;
	mesh_cld.push_back(Eigen2PointXYZRGB(t,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 - vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 + vup/2.0,rgb));
	mesh_cld.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 - vup/2.0,rgb));

	bool	bShowCam;
	int	ipolygon[18] = {0,1,2, 0,3,1, 0,4,3, 0,2,4, 3,1,4, 2,4,1};

	//TODO Mutex acquire
	pcl::PolygonMesh pm;
	pm.polygons.resize(6);
	for(int i=0;i<6;i++)
		for(int _v=0;_v<3;_v++)
			pm.polygons[i].vertices.push_back(ipolygon[i*3 + _v]);
	//pcl::toROSMsg(mesh_cld, pm.cloud);
	pcl::toPCLPointCloud2(mesh_cld, pm.cloud);
	bShowCam = true;
	std::deque<std::pair<std::string,pcl::PolygonMesh> > cam_meshes;
	std::deque<std::pair<std::string,std::vector<Eigen::Matrix<float,6,1> > > > linesToShow;

	cam_meshes.push_back(std::make_pair("camera",pm));
	//TODO mutex release

	linesToShow.push_back(std::make_pair("camera",AsVector(Eigen2Eigen(t,rgb),Eigen2Eigen(t + vforward*3.0,rgb))));
}

void Visualizer::visualizerShowCamera(cv::Matx33f& R, cv::Vec3f& t, float r, float g, float b) {
	visualizerShowCamera2(Eigen::Matrix<float,3,3,Eigen::RowMajor>(R.val),Eigen::Vector3f(t.val),r,g,b);
}
*/

/*
void StructFromMotion::meshingPointCloud(){

  pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("Meshing",true);
  viewer.setPosition(0,0);
  viewer.setSize(800,600);

  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ> ());

  // Fill in the cloud data
  cloudPCL->width    = nReconstructionCloud.size();
  cloudPCL->height   = 1;
  cloudPCL->is_dense = false;
  cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

  for (size_t i = 0; i < cloudPCL->points.size (); ++i){
     Point3D pt3d = nReconstructionCloud[i];
     cloudPCL->points[i].x = pt3d.pt.x;
     cloudPCL->points[i].y = pt3d.pt.y;
     cloudPCL->points[i].z = pt3d.pt.z;
  }

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 255, 255, 255);
  //We add the point cloud to the viewer and pass the color handler


  // Normal estimation*
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   tree->setInputCloud (cloudPCL);
   n.setInputCloud (cloudPCL);
   n.setSearchMethod (tree);
   n.setKSearch (20);
   n.compute (*normals);
   //* normals should not contain the point normals + surface curvatures

   // Concatenate the XYZ and normal fields*
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
   pcl::concatenateFields (*cloudPCL, *normals, *cloud_with_normals);
   //*cloud_with_normals = cloudPCL + normals;

   // Create search tree*
   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
   tree2->setInputCloud (cloud_with_normals);

   // Initialize objects
   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
   pcl::PolygonMesh triangles;

   // Set the maximum distance between connected points (maximum edge length)
   gp3.setSearchRadius (50);

   // Set typical values for the parameters
   gp3.setMu (2.5);
   gp3.setMaximumNearestNeighbors (100);
   gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
   gp3.setMinimumAngle(M_PI/18); // 10 degrees
   gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
   gp3.setNormalConsistency(false);

   // Get result
   gp3.setInputCloud (cloud_with_normals);
   gp3.setSearchMethod (tree2);
   gp3.reconstruct (triangles);

   // Additional vertex information
   std::vector<int> parts = gp3.getPartIDs();
   std::vector<int> states = gp3.getPointStates();


    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudPCL,normals,10,0.05,"norm");
    viewer.addPolygonMesh(triangles,"meshing");

   while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed



       viewer.spin();
   }

}
*/
