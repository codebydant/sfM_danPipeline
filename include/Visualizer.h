#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <opencv2/opencv.hpp>
#include <QWidget>

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


private:
  Ui::Visualizer *ui;

  QImage _qimage;
  cv::Mat _tmp;

};

#endif // VISUALIZER_HPP
