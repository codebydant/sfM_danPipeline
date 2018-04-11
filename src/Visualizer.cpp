#include "include/Visualizer.h"
#include "ui_Visualizer.h"

Visualizer::Visualizer(QWidget *parent) :
  QWidget(parent),ui(new Ui::Visualizer){
  ui->setupUi(this);
}

Visualizer::~Visualizer(){
  delete ui;
}





