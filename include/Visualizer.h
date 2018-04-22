#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <QWidget>
#include "PCL_visualizer.h"

namespace Ui {
  class Visualizer;
}

class Visualizer : public QWidget
{
  Q_OBJECT

public:
  explicit Visualizer(QWidget *parent = 0);
  ~Visualizer();

public slots:

  void showImage(const cv::Mat& image);
  void showPCL_Viewer();


private:
  Ui::Visualizer *ui;
   pcl::visualization::PCLVisualizer viewer=pcl::visualization::PCLVisualizer("3D Reconstruction",true);

  QImage _qimage;
  cv::Mat _tmp;

};

#endif // VISUALIZER_HPP
