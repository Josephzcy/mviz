// Copyright 2023 MINIEYE
#ifndef PB_TO_ROSMSG_HPP_
#define PB_TO_ROSMSG_HPP_

#include "ap_map_response.pb.h"
#include "apa_state.pb.h"
#include "mviz_apa_show/ApMapResponse.h"
#include "mviz_apa_show/ApaStateControl.h"
#include "mviz_apa_show/Planning.h"
#include "mviz_apa_show/PlanningToHMI.h"
#include "mviz_apa_show/UssRadar.h"
#include "mviz_apa_show/VehicleControl.h"
#include "mviz_apa_show/VehicleSignal.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
#include "vehicle_control.pb.h"
#include "vehicle_signal.pb.h"

namespace mviz1::data_conversion {

void ControlData2RosMsg(const minieye::VehicleControl& vehicle_control_pb,
                        mviz_apa_show::VehicleControl& vehicle_control_rosmsg) {
  vehicle_control_rosmsg.timestamp = vehicle_control_pb.timestamp();
  vehicle_control_rosmsg.tick = vehicle_control_pb.tick();
  vehicle_control_rosmsg.control_state = vehicle_control_pb.control_state();
  vehicle_control_rosmsg.wheel_cmd = vehicle_control_pb.wheel_cmd();
  vehicle_control_rosmsg.speed_cmd = vehicle_control_pb.speed_cmd();
  vehicle_control_rosmsg.gear_mode = vehicle_control_pb.gear_mode();
  vehicle_control_rosmsg.dist_cmd = vehicle_control_pb.dist_cmd();
  vehicle_control_rosmsg.is_reach_pathend = vehicle_control_pb.is_reach_pathend();
  vehicle_control_rosmsg.l_bias_err = vehicle_control_pb.l_bias_err();
  vehicle_control_rosmsg.yaw_bias_err = vehicle_control_pb.yaw_bias_err();
}

#ifdef MVIZ_TDA4
void APAStateControl2RosMsg(const minieye::APAStateControl& apa_state_control_pb,
                            mviz_apa_show::ApaStateControl& apa_state_control_rosmsg) {
  apa_state_control_rosmsg.settings_type.clear();
  apa_state_control_rosmsg.state = apa_state_control_pb.state();
  apa_state_control_rosmsg.timestamp = apa_state_control_pb.timestamp();
  apa_state_control_rosmsg.tick = apa_state_control_pb.tick();
  apa_state_control_rosmsg.pilot_apa_mode = apa_state_control_pb.pilot_apa_mode();

  auto setting_type = apa_state_control_pb.settings();
  for (auto setting : setting_type) {
    apa_state_control_rosmsg.settings_type.push_back(setting.second);
  }
}
void VehicleSignal2RosMsg(const minieye::VehicleSignal& vehicle_signal_pb,
                          mviz_apa_show::VehicleSignal& vehicle_signal_rosmsg) {
  vehicle_signal_rosmsg.signalsvalue.clear();
  vehicle_signal_rosmsg.signals.clear();
  vehicle_signal_rosmsg.timestamp = vehicle_signal_pb.timestamp();
  vehicle_signal_rosmsg.tick = vehicle_signal_pb.tick();
  auto begin_iter = vehicle_signal_pb.signals().begin();
  auto end_iter = vehicle_signal_pb.signals().end();
  for (auto iter = begin_iter; iter != end_iter; iter++) {
    mviz_apa_show::SignalType signal_type;
    signal_type.signaltype.value = iter->first;
    vehicle_signal_rosmsg.signalsvalue.push_back(iter->second);
    vehicle_signal_rosmsg.signals.push_back(signal_type);
  }
}
void PlanToRosMsg(const minieye::Planning& plan, mviz_apa_show::Planning& plan_rosmsg) {
  plan_rosmsg.planning_status = plan.planning_status();

}

void ApmapResponseToRosMsg(const minieye::parking::ApMapResponse& apmap_response,
                           mviz_apa_show::ApMapResponse& apmap_response_rosmsg) {
  apmap_response_rosmsg.ap_response = apmap_response.ap_response_type();
  apmap_response_rosmsg.localization_accuracy = apmap_response.localization_accuracy();
  apmap_response_rosmsg.fail_reason = apmap_response.fail_reson();

}

void PlanningToHmi2RosMsg(const minieye::PlanningToHMI& planning_to_hmi_pb,
                          mviz_apa_show::PlanningToHMI& planning_to_hmi_rosmsg) {
  planning_to_hmi_rosmsg.timestamp = planning_to_hmi_pb.timestamp();
  planning_to_hmi_rosmsg.tick = planning_to_hmi_pb.tick();
  planning_to_hmi_rosmsg.current_stage = planning_to_hmi_pb.current_stage();
  planning_to_hmi_rosmsg.remain_distance = planning_to_hmi_pb.remain_distance();
  planning_to_hmi_rosmsg.gear = planning_to_hmi_pb.gear();
  auto park_id = planning_to_hmi_pb.park_id().id_list();
  for (auto recId : park_id) {
    planning_to_hmi_rosmsg.recommand_id.push_back(recId.first);
    planning_to_hmi_rosmsg.slot_state.push_back(recId.second);
  }
}
#endif

void UssRadar2RosMsg(const minieye::UltrasonicRadar& uss_radar, mviz_apa_show::UssRadar& uss_radar_rosmsg) {
  uss_radar_rosmsg.short_radar_fl = uss_radar.short_radar_fl();
  uss_radar_rosmsg.short_radar_flm = uss_radar.short_radar_flm();
  uss_radar_rosmsg.short_radar_fr = uss_radar.short_radar_fr();
  uss_radar_rosmsg.short_radar_frm = uss_radar.short_radar_frm();

  uss_radar_rosmsg.short_radar_bl = uss_radar.short_radar_bl();
  uss_radar_rosmsg.short_radar_blm = uss_radar.short_radar_blm();
  uss_radar_rosmsg.short_radar_brm = uss_radar.short_radar_brm();
  uss_radar_rosmsg.short_radar_br = uss_radar.short_radar_br();

  uss_radar_rosmsg.long_radar_fl = uss_radar.long_radar_fl();
  uss_radar_rosmsg.long_radar_bl = uss_radar.long_radar_bl();

  uss_radar_rosmsg.long_radar_fr = uss_radar.long_radar_fr();
  uss_radar_rosmsg.long_radar_br = uss_radar.long_radar_br();
}

#ifdef MVIZ_J3
void APAStateControl2RosMsg(const minieye::APAStateControl& apa_state_control_pb,
                            mviz_apa_show::ApaStateControl& apa_state_control_rosmsg) {
  apa_state_control_rosmsg.settings_type.clear();
  apa_state_control_rosmsg.state = apa_state_control_pb.state();
  apa_state_control_rosmsg.timestamp = apa_state_control_pb.timestamp();
  apa_state_control_rosmsg.tick = apa_state_control_pb.tick();
  apa_state_control_rosmsg.pilot_apa_mode = apa_state_control_pb.pilot_apa_mode();

  auto setting_type = apa_state_control_pb.settings();
  for (auto setting : setting_type) {
    apa_state_control_rosmsg.settings_type.push_back(setting.settingtype());
  }
}
void VehicleSignal2RosMsg(const minieye::VehicleSignal& vehicle_signal_pb,
                          mviz_apa_show::VehicleSignal& vehicle_signal_rosmsg) {
  vehicle_signal_rosmsg.signalsvalue.clear();
  vehicle_signal_rosmsg.signals.clear();
  vehicle_signal_rosmsg.timestamp = vehicle_signal_pb.timestamp();
  vehicle_signal_rosmsg.tick = vehicle_signal_pb.tick();
  auto begin_iter = vehicle_signal_pb.signals().begin();
  auto end_iter = vehicle_signal_pb.signals().end();
  for (auto iter = begin_iter; iter != end_iter; iter++) {
    mviz_apa_show::SignalType signal_type;
    signal_type.signaltype.value = iter->key();
    vehicle_signal_rosmsg.signalsvalue.push_back(iter->value());
    vehicle_signal_rosmsg.signals.push_back(signal_type);
  }
}
void PlanningToHmi2RosMsg(const minieye::PlanningToHMI& planning_to_hmi_pb,
                          mviz_apa_show::PlanningToHMI& planning_to_hmi_rosmsg) {
  planning_to_hmi_rosmsg.timestamp = planning_to_hmi_pb.timestamp();
  planning_to_hmi_rosmsg.tick = planning_to_hmi_pb.tick();
  planning_to_hmi_rosmsg.current_stage = planning_to_hmi_pb.current_stage();
  planning_to_hmi_rosmsg.remain_distance = planning_to_hmi_pb.remain_distance();
  planning_to_hmi_rosmsg.gear = planning_to_hmi_pb.gear();
  auto park_id = planning_to_hmi_pb.park_id().id_list();
  for (auto recId : park_id) {
    planning_to_hmi_rosmsg.recommand_id.push_back(recId.key());
    planning_to_hmi_rosmsg.slot_state.push_back(recId.value());
  }
}
#endif

}  // namespace mviz1::data_conversion
#endif
