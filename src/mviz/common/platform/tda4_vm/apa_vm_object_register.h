#ifndef APA_VM_OBJECT_REGISTER_H_
#define APA_VM_OBJECT_REGISTER_H_

#include "tda4_vm/apa_vm_defination.h"
REGISTER(vehicle_signal, minieye::VehicleSignal);
REGISTER(odo_vehicle_signal, minieye::OdoVehicleSignal);
REGISTER(apa_vehicle_signal, minieye::ApaVehicleSignal);

REGISTER(imu, minieye::ImuDataList);
REGISTER(imu_correct_data, minieye::ImuCorrPhyData);
REGISTER(apa_gnss, minieye::GnssData);

REGISTER(uss, minieye::UltrasonicRadar);

REGISTER(apa_state_control, minieye::APAStateControl);
REGISTER(odometry_3d, minieye::Odometry3D);

REGISTER(raw_ins_ps, perception::RawInsParkingSpace);
REGISTER(freespace, freespacepoints::FreespacePoints);
REGISTER(gridmap, perception::FreespaceObstacles);
REGISTER(parkingspace, perception::ParkingSpace);
REGISTER(uss_parkingspace_mviz, perception::ParkingSpace);
REGISTER(cus_parkingspace, perception::ParkingSpace);
REGISTER(vis_parkingspace, perception::ParkingSpace);
REGISTER(vis_parkingspace_to_ui, perception::ParkingSpace);
REGISTER(parkingspace_to_ui, perception::ParkingSpace);


REGISTER(parking_object, minieye::parking::ObjectList);
REGISTER(parking_target, minieye::parking::ObjectTrackList);
REGISTER(parking_track, minieye::parking::ObjectTrackList);

REGISTER(planning, minieye::Planning);
REGISTER(planning_to_hmi, minieye::PlanningToHMI);
REGISTER(vehicle_control, minieye::VehicleControl);

REGISTER(avm_settings, minieye::AVMSettings);
REGISTER(hmi_to_soc, minieye::HmiToSoc);
REGISTER(ihu_to_soc, minieye::IHUToSoc);
REGISTER(soc_to_ihu, minieye::SocToIHU);

REGISTER(odometry_3d_gt, minieye::Odometry3D);
REGISTER(odometry_3d_lz, minieye::Odometry3D);
REGISTER(parking_ins, minieye::parking::AsensingINSData);
REGISTER(parking_gnss, minieye::parking::AsensingGNSSData);

REGISTER(vtr, minieye::Vtr);
REGISTER(ihu_to_avm, minieye::IHUToAVM);
REGISTER(apa_hmi_signal, minieye::APAHmiSignal);
REGISTER(apa_hmi_signal_res, minieye::APAHmiSignalRes);
#endif