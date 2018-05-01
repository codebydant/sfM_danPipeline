#include "include/Visualizer.h"
#include "ui_Visualizer.h"


 Visualizer::Visualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Visualizer)
  {

    ui->setupUi(this);
    this->setWindowTitle ("PERCEPTION ROBOTIC SYSTEM");
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));

    // Set up the QVTK window
    ui->qvtkwidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkwidget->GetInteractor (), ui->qvtkwidget->GetRenderWindow ());
    viewer->setBackgroundColor(1.0, 0.5, 0.5);
    viewer->addCoordinateSystem(1,"ucs",0);
    ui->qvtkwidget->update();

 }

 bool Visualizer::addPointCloudtoVisualizer(const std::vector<Point3D>& inputCloud){

   ui->qvtkwidget->SetRenderWindow (viewer->getRenderWindow ());
   viewer->setupInteractor (ui->qvtkwidget->GetInteractor (), ui->qvtkwidget->GetRenderWindow ());
    ui->qvtkwidget->update();
   if(inputCloud.empty()){
       return false;
     }

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ> ());

   //Fill in the cloud data
   cloudPCL->width    = inputCloud.size();
   cloudPCL->height   = 1;
   cloudPCL->is_dense = false;
   cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

   for(size_t i = 0; i < cloudPCL->points.size (); ++i){
         Point3D pt3d = inputCloud[i];
         cloudPCL->points[i].x = pt3d.pt.x;
         cloudPCL->points[i].y = pt3d.pt.y;
         cloudPCL->points[i].z = pt3d.pt.z;
   }

   // Define R,G,B colors for the point cloud
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 0, 255, 0);
   //We add the point cloud to the viewer and pass the color handler
   viewer->addPointCloud<pcl::PointXYZ>(cloudPCL,cloud_color,"cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");

   viewer->resetCamera();
   ui->qvtkwidget->update();
   return true;

}

 void Visualizer::updatePointCloudVisualizer(const std::vector<Point3D>& newCloud){
  // viewer->removeAllPointClouds();
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ> ());

   //Fill in the cloud data
   cloudPCL->width    = newCloud.size();
   cloudPCL->height   = 1;
   cloudPCL->is_dense = false;
   cloudPCL->points.resize(cloudPCL->width * cloudPCL->height);

   for(size_t i = 0; i < cloudPCL->points.size (); ++i){
         Point3D pt3d = newCloud[i];
         cloudPCL->points[i].x = pt3d.pt.x;
         cloudPCL->points[i].y = pt3d.pt.y;
         cloudPCL->points[i].z = pt3d.pt.z;
   }

   // Define R,G,B colors for the point cloud
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudPCL, 0, 255, 0);
   //We add the point cloud to the viewer and pass the color handler

   viewer->updatePointCloud<pcl::PointXYZ>(cloudPCL,cloud_color,"cloud");
 //  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"cloud");
  // viewer->resetCamera();
   ui->qvtkwidget->update();

 }

  Visualizer::~Visualizer()
  {
    delete ui;
  }




