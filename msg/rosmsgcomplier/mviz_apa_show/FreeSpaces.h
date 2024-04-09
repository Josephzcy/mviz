// Generated by gencpp from file mviz_apa_show/FreeSpaces.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_FREESPACES_H
#define MVIZ_APA_SHOW_MESSAGE_FREESPACES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/FreeZone.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct FreeSpaces_
{
  typedef FreeSpaces_<ContainerAllocator> Type;

  FreeSpaces_()
    : freezone()
    , dist_std(0.0)
    , angle_std(0.0)
    , height_std(0.0)  {
    }
  FreeSpaces_(const ContainerAllocator& _alloc)
    : freezone(_alloc)
    , dist_std(0.0)
    , angle_std(0.0)
    , height_std(0.0)  {
  (void)_alloc;
    }



   typedef std::vector< ::mviz_apa_show::FreeZone_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::FreeZone_<ContainerAllocator> >::other >  _freezone_type;
  _freezone_type freezone;

   typedef float _dist_std_type;
  _dist_std_type dist_std;

   typedef float _angle_std_type;
  _angle_std_type angle_std;

   typedef float _height_std_type;
  _height_std_type height_std;





  typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> const> ConstPtr;

}; // struct FreeSpaces_

typedef ::mviz_apa_show::FreeSpaces_<std::allocator<void> > FreeSpaces;

typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaces > FreeSpacesPtr;
typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaces const> FreeSpacesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::FreeSpaces_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::FreeSpaces_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeSpaces_<ContainerAllocator2> & rhs)
{
  return lhs.freezone == rhs.freezone &&
    lhs.dist_std == rhs.dist_std &&
    lhs.angle_std == rhs.angle_std &&
    lhs.height_std == rhs.height_std;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::FreeSpaces_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeSpaces_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a4296e43d4d5ba0cd20e5b46c4691635";
  }

  static const char* value(const ::mviz_apa_show::FreeSpaces_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa4296e43d4d5ba0cULL;
  static const uint64_t static_value2 = 0xd20e5b46c4691635ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/FreeSpaces";
  }

  static const char* value(const ::mviz_apa_show::FreeSpaces_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "FreeZone[] freezone                                   # 采样点集 字节数:64*n 最大容量:100\n"
"float32 dist_std                                               # 采样点距离标准差 类型:float 字节数:4 取值范围(0~FLT_MAX)\n"
"float32 angle_std                                              # 采样点角度标准差 类型:float 字节数:4 取值范围(0~FLT_MAX)\n"
"float32 height_std                                             # 采样点高度标准差 类型:float 字节数:4 取值范围(0~FLT_MAX)\n"
"================================================================================\n"
"MSG: mviz_apa_show/FreeZone\n"
"Point2f point_image_coord              # 采样点图像坐标  字节数:16 取值范围(w>=x>=0, h>=y>=0)\n"
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

  static const char* value(const ::mviz_apa_show::FreeSpaces_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.freezone);
      stream.next(m.dist_std);
      stream.next(m.angle_std);
      stream.next(m.height_std);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FreeSpaces_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::FreeSpaces_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::FreeSpaces_<ContainerAllocator>& v)
  {
    s << indent << "freezone[]" << std::endl;
    for (size_t i = 0; i < v.freezone.size(); ++i)
    {
      s << indent << "  freezone[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::FreeZone_<ContainerAllocator> >::stream(s, indent + "    ", v.freezone[i]);
    }
    s << indent << "dist_std: ";
    Printer<float>::stream(s, indent + "  ", v.dist_std);
    s << indent << "angle_std: ";
    Printer<float>::stream(s, indent + "  ", v.angle_std);
    s << indent << "height_std: ";
    Printer<float>::stream(s, indent + "  ", v.height_std);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_FREESPACES_H
