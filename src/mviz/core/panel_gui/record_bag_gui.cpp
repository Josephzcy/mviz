#include "panel_gui/record_bag_gui.h"

#include <QDebug>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>

namespace mviz_apa {
RecordBagGui::RecordBagGui(QWidget* parent) : rviz::Panel(parent) {
  btn_start_ = new QPushButton(this);
  btn_start_->setText("Start");
  connect(btn_start_, SIGNAL(clicked()), this, SLOT(RecordStart()));

  btn_stop_ = new QPushButton("Stop", this);
  // btn_stop_->setText("Stop");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(RecordStop()));
  // btn_stop_->move(130,5); //将按钮移动到某个位置
  btn_stop_->resize(40, 20);

  btn_close_ = new QPushButton;
  btn_close_->setParent(this);
  btn_close_->setText("close");
  //函数指针
  // void(QPushButton:: *click)(bool) = &QPushButton::clicked;
  // auto click = &QPushButton::clicked;
  // connect(btn_close_, click, this, &rviz::Panel::close);
  // connect(btn_close_, &QPushButton::clicked, this, &QWidget::close);
  // connect(btn_close_, &QPushButton::clicked, this, &rviz::Panel::close);
  connect(btn_close_, &QPushButton::clicked, this, [=]() { this->close(); });

  // // btn_close_不管是[=]还是[&]，setText都可以修改成功。
  // // i在[=]情况不能修改i的值，用mutable修饰后可以修改i的拷贝。
  // int i = 10;
  // [&](){
  //   btn_close_->setText("close&&&&&");
  //   i++;
  //   qDebug()<<"& "<<i; //11
  // }();//最后加()才会执行匿名函数
  // [=]() mutable {
  //   btn_close_->setText("close=====");
  //   i++;
  //   qDebug()<<"= "<<i; //12
  // }();
  // qDebug()<<" "<<i; //11

  // QString转char*
  // QString aa;
  // aa.toUtf8().data();

  btn_speed_ = new QPushButton;
  btn_speed_->setParent(this);
  btn_speed_->setText("speed");

  auto* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_start_);
  hlayout1->addWidget(btn_stop_);

  auto* hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(btn_close_);
  hlayout2->addWidget(btn_speed_);

  auto* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addLayout(hlayout2);
  setLayout(layout);

  setWindowTitle("RvizRecordBagGui");
  resize(200, 20);
  setFixedHeight(60);

  btn_start_->setEnabled(true);
  btn_stop_->setEnabled(true);
}

void RecordBagGui::RecordStart() {
  ROS_DEBUG_STREAM_NAMED("recode_gui", "start");
  ROS_INFO_STREAM("recode_gui start");
  qDebug("recode_gui start");
  std::system("pwd > /root/mviz_apa/ls.txt");
}

void RecordBagGui::RecordStop() { ROS_DEBUG_STREAM_NAMED("recode_gui", "stop"); }
void RecordBagGui::save(rviz::Config config) const { rviz::Panel::save(config); }

void RecordBagGui::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // namespace mviz_apa

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_apa::RecordBagGui, rviz::Panel)