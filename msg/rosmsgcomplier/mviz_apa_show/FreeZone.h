// Generated by gencpp from file mviz_apa_show/FreeZone.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_FREEZONE_H
#define MVIZ_APA_SHOW_MESSAGE_FREEZONE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Point2f.h>
#include <geometry_msgs/Point.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct FreeZone_
{
  typedef FreeZone_<ContainerAllocator> Type;

  FreeZone_()
    : point_image_coord()
    , point_vehicle_coord()
    , dist(0.0)
    , angle(0.0)
    , confidence(0.0)
    , camera_id(0)
    , sensor_type(0)  {
    }
  FreeZone_(const ContainerAllocator& _alloc)
    : point_image_coord(_alloc)
    , point_vehicle_coord(_alloc)
    , dist(0.0)
    , angle(0.0)
    , confidence(0.0)
    , camera_id(0)
    , sensor_type(0)  {
  (void)_alloc;
    }



   typedef  ::mviz_apa_show::Point2f_<ContainerAllocator>  _point_image_coord_type;
  _point_image_coord_type point_image_coord;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_vehicle_coord_type;
  _point_vehicle_coord_type point_vehicle_coord;

   typedef float _dist_type;
  _dist_type dist;

   typedef float _angle_type;
  _angle_type angle;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef uint32_t _camera_id_type;
  _camera_id_type camera_id;

   typedef uint32_t _sensor_type_type;
  _sensor_type_type sensor_type;





  typedef boost::shared_ptr< ::mviz_apa_show::FreeZone_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::FreeZone_<ContainerAllocator> const> ConstPtr;

}; // struct FreeZone_

typedef ::mviz_apa_show::FreeZone_<std::allocator<void> > FreeZone;

typedef boost::shared_ptr< ::mviz_apa_show::FreeZone > FreeZonePtr;
typedef boost::shared_ptr< ::mviz_apa_show::FreeZone const> FreeZoneConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::FreeZone_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::FreeZone_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::FreeZone_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeZone_<ContainerAllocator2> & rhs)
{
  return lhs.point_image_coord == rhs.point_image_coord &&
    lhs.point_vehicle_coord == rhs.point_vehicle_coord &&
    lhs.dist == rhs.dist &&
    lhs.angle == rhs.angle &&
    lhs.confidence == rhs.confidence &&
    lhs.camera_id == rhs.camera_id &&
    lhs.sensor_type == rhs.sensor_type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::FreeZone_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeZone_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeZone_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeZone_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeZone_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eefae041bf3a89bfd6f4e66115d9a3b8";
  }

  static const char* value(const ::mviz_apa_show::FreeZone_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeefae041bf3a89bfULL;
  static const uint64_t static_value2 = 0xd6f4e66115d9a3b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/FreeZone";
  }

  static const char* value(const ::mviz_apa_show::FreeZone_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Point2f point_image_coord              # 采样点图像坐标  字节数:16 取值范围(w>=x>=0, h>=y>=0)\n"
"geometry_msgs/Point point_vehicle_coord            # 采样点世界坐标  字节数:24 取值范围(100>=x>=0, 20>=y>=-20)\n"
"float32 dist                                               # 采样点与本车距离 类型:float32 字节数:4 取值范围(0~100) \n"
"float32 angle                                              # 采样点与本车前进方向角度(弧度) 类型:float32 字节数:4 取值范围(-π/2~π/2)\n"
"#LaneIdx lane_index                                      # 采样点所属车道 类型:int32 字节数:4 取值范围(0~7)\n"
"float32 confidence                                         # 置信度 类型:float32 字节数:4 取值范围(0~1)\n"
"#SpaceType type                                           # 采样点邻近不可同行类型 类型:int 字节数:4 取值范围(0~7)\n"
"#MotionProp motion_prop                                   # 采样点运动属性 类型:int 字节数:4 取值范围(0~2)\n"
"uint32 camera_id                                         # 摄像头id\n"
"uint32 sensor_type                                      # (0-camera, 1-radar, 2-LIDAR, 3-Hdmap, 4-LIDAR_CAM_FUSION)\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::mviz_apa_show::FreeZone_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point_image_coord);
      stream.next(m.point_vehicle_coord);
      stream.next(m.dist);
      stream.next(m.angle);
      stream.next(m.confidence);
      stream.next(m.camera_id);
      stream.next(m.sensor_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FreeZone_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::FreeZone_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::FreeZone_<ContainerAllocator>& v)
  {
    s << indent << "point_image_coord: ";
    s << std::endl;
    Printer< ::mviz_apa_show::Point2f_<ContainerAllocator> >::stream(s, indent + "  ", v.point_image_coord);
    s << indent << "point_vehicle_coord: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point_vehicle_coord);
    s << indent << "dist: ";
    Printer<float>::stream(s, indent + "  ", v.dist);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "camera_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.camera_id);
    s << indent << "sensor_type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sensor_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_FREEZONE_H
