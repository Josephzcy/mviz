// Generated by gencpp from file mviz_apa_show/SingleTarget.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_SINGLETARGET_H
#define MVIZ_APA_SHOW_MESSAGE_SINGLETARGET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/FusionRect3D.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct SingleTarget_
{
  typedef SingleTarget_<ContainerAllocator> Type;

  SingleTarget_()
    : id(0)
    , status(0)
    , movement(0)
    , object_class(0)
    , accel_lat_obj(0.0)
    , vis_track_id(0)
    , radar_track_id(0)
    , width(0.0)
    , confidence(0.0)
    , cipv(false)
    , v_long_obj(0.0)
    , v_lat_obj(0.0)
    , l_long_rel(0.0)
    , l_lat_rel(0.0)
    , detection_sensor(0)
    , accel_long_obj(0.0)
    , length(0.0)
    , heading_angle(0.0)
    , DetectSensor_History(0)
    , ClassConfidence(0.0)
    , TimeCreation(0)
    , LastUpdatedTime(0)
    , SensorID()
    , MotionPatternHistory(0)
    , BrakeLightSt(0)
    , TurnLightSt(0)
    , NearSide(0)
    , RectInfo()
    , OrientationStdDev(0.0)
    , ttc(0.0)
    , lane_id(0)
    , tsl_id(0)
    , cut_state(0)
    , collip_prob(0.0)  {
    }
  SingleTarget_(const ContainerAllocator& _alloc)
    : id(0)
    , status(0)
    , movement(0)
    , object_class(0)
    , accel_lat_obj(0.0)
    , vis_track_id(0)
    , radar_track_id(0)
    , width(0.0)
    , confidence(0.0)
    , cipv(false)
    , v_long_obj(0.0)
    , v_lat_obj(0.0)
    , l_long_rel(0.0)
    , l_lat_rel(0.0)
    , detection_sensor(0)
    , accel_long_obj(0.0)
    , length(0.0)
    , heading_angle(0.0)
    , DetectSensor_History(0)
    , ClassConfidence(0.0)
    , TimeCreation(0)
    , LastUpdatedTime(0)
    , SensorID(_alloc)
    , MotionPatternHistory(0)
    , BrakeLightSt(0)
    , TurnLightSt(0)
    , NearSide(0)
    , RectInfo(_alloc)
    , OrientationStdDev(0.0)
    , ttc(0.0)
    , lane_id(0)
    , tsl_id(0)
    , cut_state(0)
    , collip_prob(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;

   typedef uint32_t _status_type;
  _status_type status;

   typedef uint32_t _movement_type;
  _movement_type movement;

   typedef uint32_t _object_class_type;
  _object_class_type object_class;

   typedef float _accel_lat_obj_type;
  _accel_lat_obj_type accel_lat_obj;

   typedef uint32_t _vis_track_id_type;
  _vis_track_id_type vis_track_id;

   typedef uint32_t _radar_track_id_type;
  _radar_track_id_type radar_track_id;

   typedef float _width_type;
  _width_type width;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef uint8_t _cipv_type;
  _cipv_type cipv;

   typedef float _v_long_obj_type;
  _v_long_obj_type v_long_obj;

   typedef float _v_lat_obj_type;
  _v_lat_obj_type v_lat_obj;

   typedef float _l_long_rel_type;
  _l_long_rel_type l_long_rel;

   typedef float _l_lat_rel_type;
  _l_lat_rel_type l_lat_rel;

   typedef uint32_t _detection_sensor_type;
  _detection_sensor_type detection_sensor;

   typedef float _accel_long_obj_type;
  _accel_long_obj_type accel_long_obj;

   typedef float _length_type;
  _length_type length;

   typedef float _heading_angle_type;
  _heading_angle_type heading_angle;

   typedef uint32_t _DetectSensor_History_type;
  _DetectSensor_History_type DetectSensor_History;

   typedef float _ClassConfidence_type;
  _ClassConfidence_type ClassConfidence;

   typedef uint64_t _TimeCreation_type;
  _TimeCreation_type TimeCreation;

   typedef uint64_t _LastUpdatedTime_type;
  _LastUpdatedTime_type LastUpdatedTime;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _SensorID_type;
  _SensorID_type SensorID;

   typedef uint32_t _MotionPatternHistory_type;
  _MotionPatternHistory_type MotionPatternHistory;

   typedef uint32_t _BrakeLightSt_type;
  _BrakeLightSt_type BrakeLightSt;

   typedef uint32_t _TurnLightSt_type;
  _TurnLightSt_type TurnLightSt;

   typedef uint32_t _NearSide_type;
  _NearSide_type NearSide;

   typedef  ::mviz_apa_show::FusionRect3D_<ContainerAllocator>  _RectInfo_type;
  _RectInfo_type RectInfo;

   typedef float _OrientationStdDev_type;
  _OrientationStdDev_type OrientationStdDev;

   typedef float _ttc_type;
  _ttc_type ttc;

   typedef uint32_t _lane_id_type;
  _lane_id_type lane_id;

   typedef uint32_t _tsl_id_type;
  _tsl_id_type tsl_id;

   typedef uint32_t _cut_state_type;
  _cut_state_type cut_state;

   typedef float _collip_prob_type;
  _collip_prob_type collip_prob;





  typedef boost::shared_ptr< ::mviz_apa_show::SingleTarget_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::SingleTarget_<ContainerAllocator> const> ConstPtr;

}; // struct SingleTarget_

typedef ::mviz_apa_show::SingleTarget_<std::allocator<void> > SingleTarget;

typedef boost::shared_ptr< ::mviz_apa_show::SingleTarget > SingleTargetPtr;
typedef boost::shared_ptr< ::mviz_apa_show::SingleTarget const> SingleTargetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::SingleTarget_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::SingleTarget_<ContainerAllocator1> & lhs, const ::mviz_apa_show::SingleTarget_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.status == rhs.status &&
    lhs.movement == rhs.movement &&
    lhs.object_class == rhs.object_class &&
    lhs.accel_lat_obj == rhs.accel_lat_obj &&
    lhs.vis_track_id == rhs.vis_track_id &&
    lhs.radar_track_id == rhs.radar_track_id &&
    lhs.width == rhs.width &&
    lhs.confidence == rhs.confidence &&
    lhs.cipv == rhs.cipv &&
    lhs.v_long_obj == rhs.v_long_obj &&
    lhs.v_lat_obj == rhs.v_lat_obj &&
    lhs.l_long_rel == rhs.l_long_rel &&
    lhs.l_lat_rel == rhs.l_lat_rel &&
    lhs.detection_sensor == rhs.detection_sensor &&
    lhs.accel_long_obj == rhs.accel_long_obj &&
    lhs.length == rhs.length &&
    lhs.heading_angle == rhs.heading_angle &&
    lhs.DetectSensor_History == rhs.DetectSensor_History &&
    lhs.ClassConfidence == rhs.ClassConfidence &&
    lhs.TimeCreation == rhs.TimeCreation &&
    lhs.LastUpdatedTime == rhs.LastUpdatedTime &&
    lhs.SensorID == rhs.SensorID &&
    lhs.MotionPatternHistory == rhs.MotionPatternHistory &&
    lhs.BrakeLightSt == rhs.BrakeLightSt &&
    lhs.TurnLightSt == rhs.TurnLightSt &&
    lhs.NearSide == rhs.NearSide &&
    lhs.RectInfo == rhs.RectInfo &&
    lhs.OrientationStdDev == rhs.OrientationStdDev &&
    lhs.ttc == rhs.ttc &&
    lhs.lane_id == rhs.lane_id &&
    lhs.tsl_id == rhs.tsl_id &&
    lhs.cut_state == rhs.cut_state &&
    lhs.collip_prob == rhs.collip_prob;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::SingleTarget_<ContainerAllocator1> & lhs, const ::mviz_apa_show::SingleTarget_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::SingleTarget_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::SingleTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::SingleTarget_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bba811df60a6dab36d750de0e25e8b5f";
  }

  static const char* value(const ::mviz_apa_show::SingleTarget_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbba811df60a6dab3ULL;
  static const uint64_t static_value2 = 0x6d750de0e25e8b5fULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/SingleTarget";
  }

  static const char* value(const ::mviz_apa_show::SingleTarget_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#ObjectID\n"
"uint32 id                                      #->trackID # 目标ID\n"
"\n"
"# /*\n"
"# \"目标的融合维护状态：\n"
"#define TRACK_STATUS_NEW 2                         ->2(New)                                                   \n"
"#define TRACK_STATUS_NEW_COASTED 3                 ->3(new Coasted)\n"
"#define TRACK_STATUS_NEW_UPDATED_RADAR 4           ->4(new updated)\n"
"#define TRACK_STATUS_NEW_UPDATED_CAMERA 5          ->4(new updated)\n"
"#define TRACK_STATUS_COASTED 6                     ->6(Coasted)\n"
"#define TRACK_STATUS_UPDATED_RADAR 9               ->5(updated)\n"
"#define TRACK_STATUS_UPDATED_MUL_RADAR 10          ->5(updated)\n"
"#define TRACK_STATUS_UPDATED_CAMERA 11             ->5(updated)\n"
"#define TRACK_STATUS_UPDATED_BOTH 12               ->5(updated)\n"
"#define TRACK_STATUS_INVALID 0                     ->0(invalid)\n"
"#define TRACK_STATUS_MERGE 7                       ->1(Merged)\n"
"#define TRACK_STATUS_MERGE_BY_FRONT_VIEW 8         ->0(invalid)\n"
"# */\n"
"#MaintenanceStatus\n"
"uint32 status                                  # 目标更新状态\n"
"\n"
"# /*\n"
"# \"目标运动状态：\n"
"#define TRACK_DYNP_UNKNOWN 0                      ->0xf(notavailable)\n"
"#define TRACK_DYNP_STATIONARY 1                   ->0(stationary)\n"
"#define TRACK_DYNP_ONCOMING 2                     ->3(oncoming)\n"
"#define TRACK_DYNP_MOVING 3                       ->2(moving)\n"
"#define TRACK_DYNP_ONCOMING_FAST 4                ->3(oncoming)\n"
"#define TRACK_DYNP_MOVING_FAST 5                  ->2(moving)\n"
"# */\n"
"#MotionPattern\n"
"uint32 movement                                # 目标运动状态\n"
"\n"
"# /*\n"
"# 障碍物类别:\n"
"# const int32_t kVehicleClassNegative = 0;          ->0(undetermined)\n"
"# const int32_t kVehicleClassCar = 1;               ->1(car)\n"
"# const int32_t kVehicleClassMiniBus = 2;           ->1(car)\n"
"# const int32_t kVehicleClassBus = 3;               ->3(bus)\n"
"# const int32_t kVehicleClassTruck = 4;             ->4(boxtruck)\n"
"# const int32_t kVehicleClassSpecial = 5;           ->5(special car)\n"
"# const int32_t kVehicleClassCnt = 6;               ->0(undetermined)\n"
"# const int32_t kCyclist = 7;                       ->7(bicycle)\n"
"# const int32_t kPedestrian = 8;                    ->6(Pedestrian)\n"
"\n"
"# */\n"
"#Class\n"
"uint32 object_class                           # 目标分类\n"
"\n"
"#AccelerationAbs.y\n"
"float32 accel_lat_obj                         #->accel_lat_obj   # 目标横向加速度\n"
"\n"
"uint32 vis_track_id                         #->vis_track_id # 关联到视觉目标ID\n"
"uint32 radar_track_id                       #  # 关联的雷达目标ID\n"
"\n"
"#RectInfo.SizeLWH.y\n"
"float32 width                                 #->Width  # 目标宽度\n"
"\n"
"#ExistenceProbability\n"
"float32 confidence                            #->confidence  # 目标置信度\n"
"\n"
"bool cipv                                   #->把cipv为true的track设置到0x400报文中  # 目标是否是关键目标        \n"
"\n"
"#VelocityAbs.x\n"
"float32 v_long_obj                            #->v_long_obj   # 目标纵向速度\n"
"\n"
"#VelocityAbs.y\n"
"float32 v_lat_obj                             #->v_lat_obj   # 目标横向速度\n"
"\n"
"float32 l_long_rel                            #->l_long_rel   # 目标纵向距离(参考点)\n"
"\n"
"float32 l_lat_rel                             #->l_lat_rel   # 目标横向距离（参考点）\n"
"\n"
"# /*\n"
"# 在当前的融合Cycle内，探测到该目标的传感器：\n"
"# Bit0:Radar Front\n"
"# Bit1:Radar FrontLeft\n"
"# Bit2:Radar FrontRight\n"
"# Bit3:Radar RearLeft\n"
"# Bit4:Radar RearRight\n"
"# Bit9:Camera FrontWideAngle\n"
"# Bit10:Camera Rear\n"
"# Bit13:Camera LeftForwardLooking\n"
"# Bit14:Camera LeftBackwardLooking\n"
"# Bit17:Camera RightForwardLooking\n"
"# Bit18:Camera RightBackwardLooking\n"
"# */\n"
"# /*\n"
"# 以下只针对1v1r\n"
"# 只有bit0->1(single tracklet);\n"
"# 只有bit9->3(vision only);\n"
"# 同时有bit0和bit9->4(tracklet and vison)\n"
"# 其他情况则均为0        \n"
"# */\n"
"#DetectSensor_Current\n"
"uint32 detection_sensor                        # 目标由哪个传感器更新\n"
"\n"
"#AccelerationAbs.x\n"
"float32 accel_long_obj                           #->accel_long_obj # 目标纵向加速度\n"
"\n"
"#RectInfo.SizeLWH.x\n"
"float32 length                                   #->length # 目标长度\n"
"\n"
"#RectInfo.Orientation\n"
"float32 heading_angle                              #->heading_angle # 目标航向角\n"
"\n"
"# RadarFrame asso_radar\n"
"# CameraFrame asso_camera \n"
"# /*\n"
"# \"在目标的生命周期内，曾经探测到该目标的传感器。\n"
"# 位域的定义同detection_sensor。\"\n"
"# */\n"
"#DetectSensor_History\n"
"uint32 DetectSensor_History                    \n"
"\n"
"# /*\n"
"# \"讨论结果：如果OBJ是Radar Only OBJ时,后续讨论类别置信度输出问题\n"
"# ==>直接赋值100，很危险，所以请Minieye使用更为合理的方式评估。\"\n"
"# */\n"
"float32 ClassConfidence\n"
"# /*\n"
"# 障碍物被识别的时间戳,us\n"
"# */\n"
"uint64 TimeCreation\n"
"# /*\n"
"# 障碍物最近更新时间,us\n"
"# */\n"
"uint64 LastUpdatedTime\n"
"# /*\n"
"# \"记载当该目标被某Sensor识别时，Sensor赋予它的ID号。\n"
"# 数组大小32，排序方法与上方DetectSensor使用的位域排序方法一致。预留的元素均填“0”\"\n"
"# */\n"
"uint32[] SensorID\n"
"# /*\n"
"# \"目标历史运动状态：\n"
"# */\n"
"uint32 MotionPatternHistory\n"
"# /*\n"
"# \"刹车灯的状态：\n"
"# 0-Unknow\n"
"# 1-Off\n"
"# 2-On\"\n"
"# */\n"
"uint32 BrakeLightSt\n"
"# /*\n"
"# \"转向信号灯状态：\n"
"# 0-Unknow\n"
"# 1-Off\n"
"# 2-Left_Flash\n"
"# 3-Right_Flash\n"
"# 4-Left_and_rihgt_Flash\"\n"
"# */\n"
"uint32 TurnLightSt\n"
"# /*\n"
"# \"近边，即传感器探测的目标的面。\n"
"# 0-FRONT;\n"
"# 1-REAR;\n"
"# 2-RIGHTSIDE;\n"
"# 3-LEFTSIDE;\"\n"
"# */\n"
"uint32 NearSide                                 #->Target_pos\n"
"\n"
"FusionRect3D RectInfo\n"
"\n"
"float32 OrientationStdDev\n"
"\n"
"float32 ttc\n"
"\n"
"uint32 lane_id\n"
"\n"
"uint32 tsl_id\n"
"\n"
"uint32 cut_state\n"
"\n"
"float32 collip_prob\n"
"================================================================================\n"
"MSG: mviz_apa_show/FusionRect3D\n"
"geometry_msgs/Point Center\n"
"geometry_msgs/Point CenterStdDev\n"
"geometry_msgs/Point SizeLWH\n"
"geometry_msgs/Point SizeStdDev\n"
"geometry_msgs/Point[] Corners\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::mviz_apa_show::SingleTarget_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.status);
      stream.next(m.movement);
      stream.next(m.object_class);
      stream.next(m.accel_lat_obj);
      stream.next(m.vis_track_id);
      stream.next(m.radar_track_id);
      stream.next(m.width);
      stream.next(m.confidence);
      stream.next(m.cipv);
      stream.next(m.v_long_obj);
      stream.next(m.v_lat_obj);
      stream.next(m.l_long_rel);
      stream.next(m.l_lat_rel);
      stream.next(m.detection_sensor);
      stream.next(m.accel_long_obj);
      stream.next(m.length);
      stream.next(m.heading_angle);
      stream.next(m.DetectSensor_History);
      stream.next(m.ClassConfidence);
      stream.next(m.TimeCreation);
      stream.next(m.LastUpdatedTime);
      stream.next(m.SensorID);
      stream.next(m.MotionPatternHistory);
      stream.next(m.BrakeLightSt);
      stream.next(m.TurnLightSt);
      stream.next(m.NearSide);
      stream.next(m.RectInfo);
      stream.next(m.OrientationStdDev);
      stream.next(m.ttc);
      stream.next(m.lane_id);
      stream.next(m.tsl_id);
      stream.next(m.cut_state);
      stream.next(m.collip_prob);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SingleTarget_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::SingleTarget_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::SingleTarget_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "status: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.status);
    s << indent << "movement: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.movement);
    s << indent << "object_class: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.object_class);
    s << indent << "accel_lat_obj: ";
    Printer<float>::stream(s, indent + "  ", v.accel_lat_obj);
    s << indent << "vis_track_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vis_track_id);
    s << indent << "radar_track_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.radar_track_id);
    s << indent << "width: ";
    Printer<float>::stream(s, indent + "  ", v.width);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "cipv: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cipv);
    s << indent << "v_long_obj: ";
    Printer<float>::stream(s, indent + "  ", v.v_long_obj);
    s << indent << "v_lat_obj: ";
    Printer<float>::stream(s, indent + "  ", v.v_lat_obj);
    s << indent << "l_long_rel: ";
    Printer<float>::stream(s, indent + "  ", v.l_long_rel);
    s << indent << "l_lat_rel: ";
    Printer<float>::stream(s, indent + "  ", v.l_lat_rel);
    s << indent << "detection_sensor: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.detection_sensor);
    s << indent << "accel_long_obj: ";
    Printer<float>::stream(s, indent + "  ", v.accel_long_obj);
    s << indent << "length: ";
    Printer<float>::stream(s, indent + "  ", v.length);
    s << indent << "heading_angle: ";
    Printer<float>::stream(s, indent + "  ", v.heading_angle);
    s << indent << "DetectSensor_History: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.DetectSensor_History);
    s << indent << "ClassConfidence: ";
    Printer<float>::stream(s, indent + "  ", v.ClassConfidence);
    s << indent << "TimeCreation: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.TimeCreation);
    s << indent << "LastUpdatedTime: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.LastUpdatedTime);
    s << indent << "SensorID[]" << std::endl;
    for (size_t i = 0; i < v.SensorID.size(); ++i)
    {
      s << indent << "  SensorID[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.SensorID[i]);
    }
    s << indent << "MotionPatternHistory: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.MotionPatternHistory);
    s << indent << "BrakeLightSt: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.BrakeLightSt);
    s << indent << "TurnLightSt: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.TurnLightSt);
    s << indent << "NearSide: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.NearSide);
    s << indent << "RectInfo: ";
    s << std::endl;
    Printer< ::mviz_apa_show::FusionRect3D_<ContainerAllocator> >::stream(s, indent + "  ", v.RectInfo);
    s << indent << "OrientationStdDev: ";
    Printer<float>::stream(s, indent + "  ", v.OrientationStdDev);
    s << indent << "ttc: ";
    Printer<float>::stream(s, indent + "  ", v.ttc);
    s << indent << "lane_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.lane_id);
    s << indent << "tsl_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.tsl_id);
    s << indent << "cut_state: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.cut_state);
    s << indent << "collip_prob: ";
    Printer<float>::stream(s, indent + "  ", v.collip_prob);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_SINGLETARGET_H
