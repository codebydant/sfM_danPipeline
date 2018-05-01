#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QMainWindow>
#include <qtimer.h>
/* PCL HEADERS */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
/* VTK HEADERS */
#include <vtkSmartPointer.h>
#include <vtkSimplePointsReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <QVTKWidget.h>
#include "Utilities.h"

using PointT=pcl::PointXYZRGBA;
using PointCloudT=pcl::PointCloud<PointT>;

namespace Ui{
 class Visualizer;
}

class Visualizer : public QMainWindow, public Utilities{
      Q_OBJECT

    public:
      explicit Visualizer(QWidget *parent = 0);
      ~Visualizer();

      bool addPointCloudtoVisualizer(const std::vector<Point3D>& inputCloud);
      void updatePointCloudVisualizer(const std::vector<Point3D>& newCloud);

    private:
      Ui::Visualizer *ui;
      pcl::visualization::PCLVisualizer::Ptr viewer;

    public:      
      std::vector<Point3D> PointCloud;

};


#endif // VISUALIZER_H
