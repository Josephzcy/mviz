#include "mviz_tool_box.h"

#include <ros/ros.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <cstdio>

namespace mviz_replay {

MvizToolBox::MvizToolBox(QWidget* parent) : rviz::Panel(parent) {
  SetUI();
  connect(m_replay_button, &QPushButton::clicked, [&]() {
    ReplayUI* replay_ui = new ReplayUI();
    replay_ui->setAttribute(Qt::WA_DeleteOnClose);
    replay_ui->setWindowTitle("Mviz Replay");
    replay_ui->show();
  });
}

void MvizToolBox::save(rviz::Config config) const { rviz::Panel::save(config); }

void MvizToolBox::load(const rviz::Config& config) { rviz::Panel::load(config); }

void MvizToolBox::SetUI() {
  m_replay_button = new QPushButton("Replay", this);
  m_record_button = new QPushButton("Record", this);
  m_other0_button = new QPushButton("...", this);
  m_other1_button = new QPushButton("...", this);
  m_replay_button->setEnabled(false);
  m_record_button->setEnabled(false);
  m_other0_button->setEnabled(false);
  m_other1_button->setEnabled(false);

  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(m_replay_button);
  hlayout1->addWidget(m_record_button);

  auto* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(m_other0_button);
  hlayout2->addWidget(m_other1_button);

  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout2);
  setLayout(layout);

  // setWindowTitle("Mviz Tool Box");
  // setFixedHeight(60);
}

}  // end of namespace mviz_replay
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_replay::MvizToolBox, rviz::Panel)