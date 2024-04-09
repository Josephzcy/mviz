#ifndef J3_PLATFORM_DEFINATION_H_
#define J3_PLATFORM_DEFINATION_H_
#include <variant>

#include "RawInsParkingSpace.pb.h"
#include "apa_odometry.pb.h"
#include "apa_state.pb.h"
#include "avm_calib_ctrl.pb.h"
#include "avm_calib_param.pb.h"
#include "freespace_obstacles.pb.h"
#include "freespacepoints.pb.h"
#include "odometry_3d.pb.h"
#include "parking_object.pb.h"
#include "parkingspace.pb.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
#include "ultra_radar.pb.h"
#include "vehicle_control.pb.h"
#include "vehicle_signal.pb.h"
#include "vtr.pb.h"

using PbType =
    std::variant<perception::RawInsParkingSpace, apa::ApaOdometry, minieye::APAStateControl, minieye::Odometry3D,
                 minieye::AVMCalibCtrl, freespacepoints::FreespacePoints, perception::FreespaceObstacles,
                 perception::ParkingSpace, minieye::Planning, minieye::PlanningToHMI, minieye::UltrasonicRadar,
                 minieye::VehicleControl, minieye::Vtr, minieye::VehicleSignal>;




#endif