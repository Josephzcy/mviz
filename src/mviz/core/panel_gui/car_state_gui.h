#ifndef CARSTATEGUI_H_
#define CARSTATEGUI_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QComboBox>
#include <QLabel>
#include <QPushButton>

#include "mviz_apa_show/ApMapResponse.h"
#include "mviz_apa_show/ApaStateControl.h"
#include "mviz_apa_show/Planning.h"
#include "mviz_apa_show/VehicleControl.h"
#include "mviz_apa_show/VehicleSignal.h"

namespace mviz_apa {

using PanelStatusMap = std::map<std::uint16_t, QString>;

class CarStateGui : public rviz::Panel {
  Q_OBJECT
 public:
  explicit CarStateGui(QWidget* parent = nullptr);
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 Q_SIGNALS:
  void getOneMsg();
 public Q_SLOTS:

 protected Q_SLOTS:
  void UpdateVehicleSignalPanel();
  void UpdateControlPanel();
  void UpdateStateControlPanel();
  void UpdatePlanPanel();
  void UpdateApResponsePanel();

 protected:
  void StateControlCallBack(const mviz_apa_show::ApaStateControl& apa_state_control_rosmsg);
  void VehicleControlCallBack(const mviz_apa_show::VehicleControl& vehicle_control_rosmsg);
  void VehicleSignalCallBack(const mviz_apa_show::VehicleSignal& vehicle_signal_rosmsg);
  void PlanningCallBack(const mviz_apa_show::Planning& planning_rosmsg);
  void ApMapResponseCallBack(const mviz_apa_show::ApMapResponse& ap_map_response_rosmsg);

  QLabel* speed_label_;
  QLabel* acc_label_;
  QLabel* gear_label_;
  QLabel* steering_angle_label_;

  QLabel* speed_value_;
  QLabel* acc_value_;
  QLabel* gear_value_;
  QLabel* steering_angle_value_;

  QLabel* pilot_mode_label_;
  QLabel* state_mode_label_;
  QLabel* shake_hand_label_;
  QLabel* hmi_info_label_;
  QLabel* plan_type_label_;

  QLabel* pilot_mode_value_;
  QLabel* state_mode_value_;

  QLabel* shake_hand_value_;
  QLabel* hmi_info_value_;
  QLabel* plan_type_value_;

  QLabel* l_bias_label_;
  QLabel* yaw_bias_label_;

  QLabel* l_bias_value_;
  QLabel* yaw_bias_value_;

  QLabel* plan_status_label_;
  QLabel* ap_response_label_;
  QLabel* localization_accuracy_label_;
  QLabel* fail_reason_label_;

  QLabel* plan_status_value_;
  QLabel* ap_response_value_;
  QLabel* localization_accuracy_value_;
  QLabel* fail_reason_value_;

  double speed_{0.0};
  double acc_{0.0};
  int gear_{0};
  double steering_angle_{0.0};

  int pilot_mode_{0};
  int state_mode_{0};
  int shake_hand_{0};
  int hmi_info_{0};
  int plan_type_{0};

  float l_bias_{0.0};
  float yaw_bias_{0.0};

  QString plan_status_;
  QString ap_response_;
  float localization_accuracy_;
  QString fail_reason_;

  ros::Subscriber subStateControl_;
  ros::Subscriber subControl_;
  ros::Subscriber subVehicleSignal_;

  ros::Subscriber sub_planning_;
  ros::Subscriber sub_ap_response_;

  ros::NodeHandle nh_;

  void InitStausTable();
  PanelStatusMap ap_respone_;
  PanelStatusMap fail_status_;
  PanelStatusMap planning_status_map_;
};

}  // namespace mviz_apa

#endif