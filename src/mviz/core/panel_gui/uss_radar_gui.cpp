// Copy 2023 Minieye
#include "panel_gui/uss_radar_gui.h"

#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

#include "mviz_apa_show/UssRadar.h"
namespace mviz_apa {

UssRadarGui::UssRadarGui(QWidget* parent) : rviz::Panel(parent) {
  short_radar_fl_ = new QLabel("short_radar_fl", this);
  short_radar_flm_ = new QLabel("short_radar_flm", this);
  short_radar_frm_ = new QLabel("short_radar_frm", this);
  short_radar_fr_ = new QLabel("short_radar_fr", this);

  short_radar_bl_ = new QLabel("short_radar_bl", this);
  short_radar_blm_ = new QLabel("short_radar_blm", this);
  short_radar_brm_ = new QLabel("short_radar_brm", this);
  short_radar_br_ = new QLabel("short_radar_br", this);

  long_radar_fl_ = new QLabel("long_radar_fl", this);
  long_radar_bl_ = new QLabel("long_radar_bl", this);

  long_radar_fr_ = new QLabel("long_radar_fr", this);
  long_radar_br_ = new QLabel("long_radar_br", this);

  // todo:value
  short_radar_fl_value_ = new QLabel("this");
  short_radar_flm_value_ = new QLabel("this");
  short_radar_frm_value_ = new QLabel("this");
  short_radar_fr_value_ = new QLabel("this");

  short_radar_bl_value_ = new QLabel("this");
  short_radar_blm_value_ = new QLabel("this");
  short_radar_brm_value_ = new QLabel("this");
  short_radar_br_value_ = new QLabel("this");

  long_radar_fl_value_ = new QLabel("this");
  long_radar_bl_value_ = new QLabel("this");

  long_radar_fr_value_ = new QLabel("this");
  long_radar_br_value_ = new QLabel("this");

  uss_radar_display_value_ = {0};

  UssRadarGuiLayout();

  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(UpdateUssRadarPanel()));
  timer->start(200);

  sub_uss_radar_ = nh_.subscribe("/qt_msg/uss_radar", 2, &UssRadarGui::UssRadarCallBack, this);
  // todo: 用定时器init signal and slots
}
void UssRadarGui::UpdateUssRadarPanel() {
  // todo:mviz_apa_show to Qt Qlabel text

  short_radar_fl_value_->setNum(uss_radar_display_value_.short_radar_fl);
  short_radar_flm_value_->setNum(uss_radar_display_value_.short_radar_flm);
  short_radar_frm_value_->setNum(uss_radar_display_value_.short_radar_frm);
  short_radar_fr_value_->setNum(uss_radar_display_value_.short_radar_fr);

  short_radar_bl_value_->setNum(uss_radar_display_value_.short_radar_bl);
  short_radar_blm_value_->setNum(uss_radar_display_value_.short_radar_blm);
  short_radar_brm_value_->setNum(uss_radar_display_value_.short_radar_brm);
  short_radar_br_value_->setNum(uss_radar_display_value_.short_radar_br);

  long_radar_fl_value_->setNum(uss_radar_display_value_.long_radar_fl);
  long_radar_bl_value_->setNum(uss_radar_display_value_.long_radar_bl);
  long_radar_fr_value_->setNum(uss_radar_display_value_.long_radar_fr);
  long_radar_br_value_->setNum(uss_radar_display_value_.long_radar_br);
}
void UssRadarGui::UssRadarCallBack(const mviz_apa_show::UssRadar& uss_radar_rosmsg) {
  uss_radar_display_value_.short_radar_fl = uss_radar_rosmsg.short_radar_fl;
  uss_radar_display_value_.short_radar_flm = uss_radar_rosmsg.short_radar_flm;
  uss_radar_display_value_.short_radar_frm = uss_radar_rosmsg.short_radar_frm;
  uss_radar_display_value_.short_radar_fr = uss_radar_rosmsg.short_radar_fr;

  uss_radar_display_value_.short_radar_bl = uss_radar_rosmsg.short_radar_bl;
  uss_radar_display_value_.short_radar_blm = uss_radar_rosmsg.short_radar_blm;
  uss_radar_display_value_.short_radar_brm = uss_radar_rosmsg.short_radar_brm;
  uss_radar_display_value_.short_radar_br = uss_radar_rosmsg.short_radar_br;

  uss_radar_display_value_.long_radar_fl = uss_radar_rosmsg.long_radar_fl;
  uss_radar_display_value_.long_radar_bl = uss_radar_rosmsg.long_radar_bl;
  uss_radar_display_value_.long_radar_fr = uss_radar_rosmsg.long_radar_fr;
  uss_radar_display_value_.long_radar_br = uss_radar_rosmsg.long_radar_br;

  // todo 订阅mviz_apa_show::UssRadar消息
}
void UssRadarGui::UssRadarGuiLayout() {
  auto* h_layout_left_front = new QHBoxLayout;
  h_layout_left_front->addWidget(short_radar_fl_);
  h_layout_left_front->addWidget(short_radar_fl_value_);
  h_layout_left_front->addWidget(short_radar_flm_);
  h_layout_left_front->addWidget(short_radar_flm_value_);

  auto* h_layout_right_front = new QHBoxLayout;
  h_layout_right_front->addWidget(short_radar_frm_);
  h_layout_right_front->addWidget(short_radar_frm_value_);
  h_layout_right_front->addWidget(short_radar_fr_);
  h_layout_right_front->addWidget(short_radar_fr_value_);

  auto* h_layout_long_left = new QHBoxLayout;
  h_layout_long_left->addWidget(long_radar_fl_);
  h_layout_long_left->addWidget(long_radar_fl_value_);
  h_layout_long_left->addWidget(long_radar_bl_);
  h_layout_long_left->addWidget(long_radar_bl_value_);

  auto* h_layout_long_right = new QHBoxLayout;
  h_layout_long_right->addWidget(long_radar_fr_);
  h_layout_long_right->addWidget(long_radar_fr_value_);
  h_layout_long_right->addWidget(long_radar_br_);
  h_layout_long_right->addWidget(long_radar_br_value_);

  auto* h_layout_rear_left = new QHBoxLayout;
  h_layout_rear_left->addWidget(short_radar_bl_);
  h_layout_rear_left->addWidget(short_radar_bl_value_);
  h_layout_rear_left->addWidget(short_radar_blm_);
  h_layout_rear_left->addWidget(short_radar_blm_value_);

  auto* h_layout_rear_right = new QHBoxLayout;
  h_layout_rear_right->addWidget(short_radar_brm_);
  h_layout_rear_right->addWidget(short_radar_brm_value_);
  h_layout_rear_right->addWidget(short_radar_br_);
  h_layout_rear_right->addWidget(short_radar_br_value_);

  auto* hlayout = new QVBoxLayout;
  hlayout->addLayout(h_layout_left_front);
  hlayout->addLayout(h_layout_right_front);
  hlayout->addLayout(h_layout_long_left);
  hlayout->addLayout(h_layout_long_right);
  hlayout->addLayout(h_layout_rear_left);
  hlayout->addLayout(h_layout_rear_right);
  setLayout(hlayout);
}

void UssRadarGui::save(rviz::Config config) const { rviz::Panel::save(config); }

void UssRadarGui::load(const rviz::Config& config) { rviz::Panel::load(config); }
}  // namespace mviz_apa

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_apa::UssRadarGui, rviz::Panel)