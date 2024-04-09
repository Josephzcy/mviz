#ifndef Mviz_REPLAY_GUI_H_
#define Mviz_REPLAY_GUI_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QDebug>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QtGlobal>

#include "global_value.h"
#include "replay_ui.h"
namespace mviz_replay {
class ReplayGui : public rviz::Panel {
  Q_OBJECT
 public:
  explicit ReplayGui(QWidget* parent = nullptr);
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;
  void SetUI();

 Q_SIGNALS:

 public Q_SLOTS:

 protected Q_SLOTS:

 protected:
 private:
  ReplayUI* m_replay_ui = nullptr;
};

}  // namespace mviz_replay

#endif