#ifndef RECORD_BAG_GUI_H__
#define RECORD_BAG_GUI_H__

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QPushButton>

class QLineEdit;
class QSpinBox;

namespace mviz_apa {
class RecordBagGui : public rviz::Panel {
  Q_OBJECT
 public:
  explicit RecordBagGui(QWidget* parent = nullptr);
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;
 public Q_SLOTS:
 protected Q_SLOTS:
  void RecordStart();
  void RecordStop();

 protected:
  QPushButton* btn_start_;
  QPushButton* btn_stop_;
  QPushButton* btn_close_;
  QPushButton* btn_speed_;
};

}  // namespace mviz_apa

#endif