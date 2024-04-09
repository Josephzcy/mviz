#ifndef HAVP_VM_DEFINATION_H_
#define HAVP_VM_DEFINATION_H_
#include <variant>

#include "RawInsParkingSpace.pb.h"
#include "ap_map_response.pb.h"
#include "apa_gnss.pb.h"
#include "apa_state.pb.h"
#include "avm_settings.pb.h"
#include "camera.pb.h"
#include "freespace_obstacles.pb.h"
#include "freespacepoints.pb.h"
#include "hmi_to_soc.pb.h"
#include "ihu_to_avm.pb.h"
#include "ihu_to_soc.pb.h"
#include "imu.pb.h"
#include "map_engine_response.pb.h"
#include "odo_vehicle_signal.pb.h"
#include "odometry_3d.pb.h"
#include "parking_gnss.pb.h"
#include "parking_ins.pb.h"
#include "parking_object.pb.h"
#include "parkingspace.pb.h"
#include "planning.pb.h"
#include "planningtohmi.pb.h"
#include "roadmarks.pb.h"
#include "soc_to_ihu.pb.h"
#include "ultra_radar.pb.h"
#include "vehicle_control.pb.h"
#include "vehicle_signal.pb.h"
#include "vtr.pb.h"

#include "ihu_to_avm.pb.h"
#include "apa_hmi_signal.pb.h"

using PbType =
    std::variant<minieye::VehicleSignal, minieye::OdoVehicleSignal, minieye::ApaVehicleSignal, minieye::ImuDataList,
                 minieye::ImuCorrPhyData, minieye::GnssData, minieye::UltrasonicRadar, minieye::APAStateControl,
                 minieye::parking::ApMapResponse, minieye::parking::MapEngineResponse, minieye::Odometry3D,
                 perception::RawInsParkingSpace, freespacepoints::FreespacePoints, perception::FreespaceObstacles,
                 perception::ParkingSpace, perception::Roadmarks, minieye::parking::ObjectList,
                 minieye::parking::ObjectTrackList, minieye::parking::AsensingINSData,
                 minieye::parking::AsensingGNSSData, minieye::Planning, minieye::PlanningToHMI, minieye::VehicleControl,
                 minieye::Vtr, minieye::HmiToSoc, minieye::IHUToSoc, minieye::AVMSettings, minieye::SocToIHU,
                 minieye::IHUToAVM, minieye::APAHmiSignal, minieye::APAHmiSignalRes>;

#endif
