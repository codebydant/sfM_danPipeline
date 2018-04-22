#include "include/Visualizer.h"
#include "ui_Visualizer.h"

Visualizer::Visualizer(QWidget *parent) :
  QWidget(parent),ui(new Ui::Visualizer){
  ui->setupUi(this);


}

Visualizer::~Visualizer(){
  delete ui;
}

void Visualizer::showImage(const cv::Mat& image) {
        // Convert the image to the RGB888 format
        switch (image.type()) {
        case CV_8UC1:
            cvtColor(image, _tmp,cv::COLOR_GRAY2RGB);
            break;
        case CV_8UC3:
            cvtColor(image, _tmp, cv::COLOR_BGR2RGB);
            break;
        }

        // QImage needs the data to be stored continuously in memory
        assert(_tmp.isContinuous());
        // Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
        // (http://qt-project.org/doc/qt-4.8/qimage.html#QImage-6) is 3*width because each pixel
        // has three bytes.
        _qimage = QImage(_tmp.data, _tmp.cols, _tmp.rows, _tmp.cols*3, QImage::Format_RGB888);






        repaint();
}

void Visualizer::showPCL_Viewer(){


}








