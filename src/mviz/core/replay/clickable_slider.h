#ifndef Mviz_CLICKABLE_SLIDER_H
#define Mviz_CLICKABLE_SLIDER_H
#include <QMouseEvent>
#include <QSlider>

class ClickableSlider : public QSlider {
 public:
  explicit ClickableSlider(QWidget* parent = nullptr);
  explicit ClickableSlider(Qt::Orientation orientation, QWidget* parent = nullptr);

 protected:
  // void mouseDoubleClickEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
};

#endif