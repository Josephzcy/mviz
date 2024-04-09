#ifndef J3_OBJECT_REGISTER_H_
#define J3_OBJECT_REGISTER_H_
#include "j3/j3_defination.h"
REGISTER(raw_ins_ps, perception::RawInsParkingSpace);
REGISTER(odometry, apa::ApaOdometry);

REGISTER(avm_calib_ctrl, minieye::AVMCalibCtrl);
REGISTER(gridmap, perception::FreespaceObstacles);
REGISTER(freespace, freespacepoints::FreespacePoints);

REGISTER(odometry_3d, minieye::Odometry3D);
REGISTER(odometry_3d_gt, minieye::Odometry3D);
REGISTER(odometry_3d_lz, minieye::Odometry3D);
REGISTER(parkingspace, perception::ParkingSpace);

REGISTER(planning, minieye::Planning);
REGISTER(planning_to_hmi, minieye::PlanningToHMI);
REGISTER(vehicle_control, minieye::VehicleControl);

REGISTER(vehicle_signal, minieye::VehicleSignal);
REGISTER(uss, minieye::UltrasonicRadar);

REGISTER(vtr, minieye::Vtr);

#endif