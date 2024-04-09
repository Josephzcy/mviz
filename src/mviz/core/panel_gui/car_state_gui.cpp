#include "panel_gui/car_state_gui.h"

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
void CarStateGui::StateControlCallBack(const mviz_apa_show::ApaStateControl& apa_state_control_rosmsg) {
  pilot_mode_ = apa_state_control_rosmsg.pilot_apa_mode;
  std::int16_t settingSize = apa_state_control_rosmsg.settings_type.size();
  if (settingSize >= 3) {
    state_mode_ = apa_state_control_rosmsg.state;
    shake_hand_ = apa_state_control_rosmsg.settings_type[settingSize - 3];
    hmi_info_ = apa_state_control_rosmsg.settings_type[settingSize - 2];
    plan_type_ = apa_state_control_rosmsg.settings_type[settingSize - 1];
  }
  // Q_EMIT getOneapa_state_control_rosmsg();
}

void CarStateGui::VehicleControlCallBack(const mviz_apa_show::VehicleControl& vehicle_control_rosmsg) {
  l_bias_ = vehicle_control_rosmsg.l_bias_err;
  yaw_bias_ = vehicle_control_rosmsg.yaw_bias_err;
}
void CarStateGui::VehicleSignalCallBack(const mviz_apa_show::VehicleSignal& vehicle_signal_rosmsg) {
  for (size_t i = 0; i < vehicle_signal_rosmsg.signals.size(); i++) {
    if (vehicle_signal_rosmsg.signals[i].signaltype.value == 0) {
      speed_ = vehicle_signal_rosmsg.signalsvalue[i];
    } else if (vehicle_signal_rosmsg.signals[i].signaltype.value == 5) {
      steering_angle_ = vehicle_signal_rosmsg.signalsvalue[i];
    } else if (vehicle_signal_rosmsg.signals[i].signaltype.value == 11) {
      gear_ = vehicle_signal_rosmsg.signalsvalue[i];
    } else if (vehicle_signal_rosmsg.signals[i].signaltype.value == 14) {
      acc_ = vehicle_signal_rosmsg.signalsvalue[i];
    }
  }
}

void CarStateGui::PlanningCallBack(const mviz_apa_show::Planning& planning_rosmsg) {
  // plan_status_ = planning_status_map_.at(static_cast<std::uint16_t>(planning_rosmsg.planning_status));
  plan_status_ = planning_status_map_[static_cast<std::uint16_t>(planning_rosmsg.planning_status)];
}

void CarStateGui::ApMapResponseCallBack(const mviz_apa_show::ApMapResponse& ap_map_response_rosmsg) {
  ap_response_ = ap_respone_[static_cast<std::uint16_t>(ap_map_response_rosmsg.ap_response)];
  localization_accuracy_ = ap_map_response_rosmsg.localization_accuracy;
  fail_reason_ = fail_status_[static_cast<std::uint16_t>(ap_map_response_rosmsg.fail_reason)];
}

void CarStateGui::UpdateStateControlPanel() {
  pilot_mode_value_->setNum(pilot_mode_);
  state_mode_value_->setNum(state_mode_);
  shake_hand_value_->setNum(shake_hand_);
  hmi_info_value_->setNum(hmi_info_);
  plan_type_value_->setNum(plan_type_);
}

void CarStateGui::UpdateVehicleSignalPanel() {
  speed_value_->setNum(speed_);
  acc_value_->setNum(acc_);
  gear_value_->setNum(gear_);
  steering_angle_value_->setNum(steering_angle_);
}
void CarStateGui::UpdateControlPanel() {
  l_bias_value_->setNum(l_bias_);
  yaw_bias_value_->setNum(yaw_bias_);
}

void CarStateGui::UpdatePlanPanel() { plan_status_value_->setText(plan_status_); }
void CarStateGui::UpdateApResponsePanel() {
  ap_response_value_->setText(ap_response_);
  localization_accuracy_value_->setNum(localization_accuracy_);
  fail_reason_value_->setText(fail_reason_);
}

CarStateGui::CarStateGui(QWidget* parent) : rviz::Panel(parent) {
  speed_label_ = new QLabel("speed:", this);
  acc_label_ = new QLabel("acc:", this);
  gear_label_ = new QLabel("gear:", this);
  steering_angle_label_ = new QLabel("steering_angle:", this);

  speed_value_ = new QLabel(this);
  acc_value_ = new QLabel(this);
  gear_value_ = new QLabel(this);
  steering_angle_value_ = new QLabel(this);

  pilot_mode_label_ = new QLabel("pilot_mode:", this);
  state_mode_label_ = new QLabel("state_mode:", this);
  shake_hand_label_ = new QLabel("shake_hand:", this);
  hmi_info_label_ = new QLabel("hmi_info:", this);
  plan_type_label_ = new QLabel("plan_type:", this);

  pilot_mode_value_ = new QLabel(this);
  state_mode_value_ = new QLabel(this);
  shake_hand_value_ = new QLabel(this);
  hmi_info_value_ = new QLabel(this);
  plan_type_value_ = new QLabel(this);

  l_bias_label_ = new QLabel("l_bias::", this);
  yaw_bias_label_ = new QLabel("yaw_bias:", this);

  l_bias_value_ = new QLabel(this);
  yaw_bias_value_ = new QLabel(this);

  plan_status_label_ = new QLabel("plan_status:", this);
  ap_response_label_ = new QLabel("ap_response_type:", this);
  localization_accuracy_label_ = new QLabel("localization_accuracy:", this);
  fail_reason_label_ = new QLabel("fail_reason:", this);

  plan_status_value_ = new QLabel(this);
  ap_response_value_ = new QLabel(this);
  localization_accuracy_value_ = new QLabel(this);
  fail_reason_value_ = new QLabel(this);

  InitStausTable();
  // speed_->setStyleSheet("QLabel{font:18px;color:black;background-color:rgb(f9,f9,f9);}");
  // acc_->setStyleSheet("QLabel{font:18px;color:black;background-color:rgb(f9,f9,f9);}");
  // gear_->setStyleSheet("QLabel{font:18px;color:black;background-color:rgb(f9,f9,f9);}");
  // steering_angle_->setStyleSheet("QLabel{font:18px;color:black;background-color:rgb(f9,f9,f9);}");

  auto* vlayout = new QVBoxLayout;
  vlayout->addWidget(speed_label_);
  vlayout->addWidget(acc_label_);
  vlayout->addWidget(gear_label_);
  vlayout->addWidget(steering_angle_label_);

  vlayout->addWidget(pilot_mode_label_);
  vlayout->addWidget(state_mode_label_);

  vlayout->addWidget(shake_hand_label_);
  vlayout->addWidget(hmi_info_label_);
  vlayout->addWidget(plan_type_label_);

  vlayout->addWidget(l_bias_label_);
  vlayout->addWidget(yaw_bias_label_);

  vlayout->addWidget(plan_status_label_);
  vlayout->addWidget(ap_response_label_);
  vlayout->addWidget(localization_accuracy_label_);
  vlayout->addWidget(fail_reason_label_);

  auto* vlayout2 = new QVBoxLayout;
  vlayout2->addWidget(speed_value_);
  vlayout2->addWidget(acc_value_);
  vlayout2->addWidget(gear_value_);
  vlayout2->addWidget(steering_angle_value_);

  vlayout2->addWidget(pilot_mode_value_);
  vlayout2->addWidget(state_mode_value_);

  vlayout2->addWidget(shake_hand_value_);
  vlayout2->addWidget(hmi_info_value_);
  vlayout2->addWidget(plan_type_value_);

  vlayout2->addWidget(l_bias_value_);
  vlayout2->addWidget(yaw_bias_value_);

  vlayout2->addWidget(plan_status_value_);
  vlayout2->addWidget(ap_response_value_);
  vlayout2->addWidget(localization_accuracy_value_);
  vlayout2->addWidget(fail_reason_value_);

  auto* hlayout = new QHBoxLayout;
  hlayout->addLayout(vlayout);
  hlayout->addLayout(vlayout2);

  setLayout(hlayout);

  QTimer* output_timer = new QTimer(this);

  connect(output_timer, SIGNAL(timeout()), this, SLOT(UpdateStateControlPanel()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(UpdateControlPanel()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(UpdateVehicleSignalPanel()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(UpdatePlanPanel()));
  connect(output_timer, SIGNAL(timeout()), this, SLOT(UpdateApResponsePanel()));
  // connect(this, &CarStateGui::getOneMsg, this, &CarStateGui::setValue);

  output_timer->start(20);

  // sub_ = nh_.subscribe("/msg/apa_state_control", 2,
  // std::bind(&CarStateGui::CallBack,this,std::placeholders::_1),this);
  subStateControl_ = nh_.subscribe("/msg/apa_state_control", 2, &CarStateGui::StateControlCallBack, this);
  subControl_ = nh_.subscribe("/msg/vehicle_control", 2, &CarStateGui::VehicleControlCallBack, this);
  subVehicleSignal_ = nh_.subscribe("/msg/vehicle_signal", 2, &CarStateGui::VehicleSignalCallBack, this);
  sub_planning_ = nh_.subscribe("/qt_msg/planning", 2, &CarStateGui::PlanningCallBack, this);
  sub_ap_response_ = nh_.subscribe("/qt_msg/apmap_response", 2, &CarStateGui::ApMapResponseCallBack, this);
}
void CarStateGui::save(rviz::Config config) const { rviz::Panel::save(config); }

void CarStateGui::load(const rviz::Config& config) { rviz::Panel::load(config); }

void CarStateGui::InitStausTable() {
  ap_respone_ = {{0, "ChooseMap"},
                 {1, "MapReadyToMapping"},
                 {2, "Mapping"},
                 {3, "MapDone"},
                 {4, "MapFail"},
                 {5, "LReadyToMapping"},
                 {6, "LReadyToLocalizing"},
                 {7, "Localization"},
                 {8, "LocalizingDone"},
                 {9, "LocalizingFailed"},
                 {10, "BackToGround"},
                 {11, "LocalizingOutOfMap"}};
  fail_status_ = {{0, "NotFail"}, {1, "BackwardTooFar"}, {2, "Speeding"}};
  planning_status_map_ = {{0, "Standby"}, {1, "Active"},   {2, "Suspend"},
                          {3, "Replan"},  {4, "Finished"}, {5, "Failure"}};
}

}  // namespace mviz_apa

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mviz_apa::CarStateGui, rviz::Panel)