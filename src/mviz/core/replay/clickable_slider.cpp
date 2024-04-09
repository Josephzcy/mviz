#include "clickable_slider.h"

ClickableSlider::ClickableSlider(QWidget* parent) : QSlider(parent) {}
ClickableSlider::ClickableSlider(Qt::Orientation orientation, QWidget* parent) : QSlider(orientation, parent) {}

// void ClickableSlider::mouseDoubleClickEvent(QMouseEvent* event) {
//   if (event->button() == Qt::LeftButton) {
//     int value = minimum() + ((maximum() - minimum()) * event->x()) / width();
//     setValue(value);
//     event->accept();
//   } else {
//     QSlider::mouseDoubleClickEvent(event);
//   }
// }

void ClickableSlider::mousePressEvent(QMouseEvent* event) {
  //先调用父类的鼠标点击处理事件，这样可以不影响拖动的情况
  QSlider::mousePressEvent(event);
  //获取鼠标的位置，这里并不能直接从event中取值（因为如果是拖动的话，鼠标开始点击的位置没有意义了）
  double pos = event->pos().x() / (double)width();
  setValue(pos * (maximum() - minimum()) + minimum());
  //发送自定义的鼠标单击信号
  // emit costomSliderClicked();
}