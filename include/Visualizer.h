#ifndef VISUALIZER_H
#define VISUALIZER_H

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

private slots:


private:
  Ui::Visualizer *ui;
};

#endif // VISUALIZER_HPP
