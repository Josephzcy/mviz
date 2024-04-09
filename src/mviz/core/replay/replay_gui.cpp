#include "replay_gui.h"

#include <ros/ros.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <cstdio>

namespace mviz_replay {

ReplayGui::ReplayGui(QWidget* parent) : rviz::Panel(parent) { SetUI(); }

void ReplayGui::save(rviz::Config config) const { rviz::Panel::save(config); }

void ReplayGui::load(const rviz::Config& config) { rviz::Panel::load(config); }

void ReplayGui::SetUI() {
  m_replay_ui = new ReplayUI(this);
  m_replay_ui->setWindowTitle("Mviz Replay");
  auto* layout = new QVBoxLayout;
  layout->addWidget(m_replay_ui);
  setLayout(layout);
}

}  // end of namespace mviz_replay
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_replay::ReplayGui, rviz::Panel)