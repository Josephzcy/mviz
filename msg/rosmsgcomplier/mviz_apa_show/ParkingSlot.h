// Generated by gencpp from file mviz_apa_show/ParkingSlot.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_PARKINGSLOT_H
#define MVIZ_APA_SHOW_MESSAGE_PARKINGSLOT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/CornerPt.h>
#include <mviz_apa_show/ParkSlotType.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct ParkingSlot_
{
  typedef ParkingSlot_<ContainerAllocator> Type;

  ParkingSlot_()
    : id(0)
    , corner_pts()
    , type()
    , occupied(false)
    , width(0.0)
    , length(0.0)  {
    }
  ParkingSlot_(const ContainerAllocator& _alloc)
    : id(0)
    , corner_pts(_alloc)
    , type(_alloc)
    , occupied(false)
    , width(0.0)
    , length(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;

   typedef std::vector< ::mviz_apa_show::CornerPt_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::CornerPt_<ContainerAllocator> >::other >  _corner_pts_type;
  _corner_pts_type corner_pts;

   typedef  ::mviz_apa_show::ParkSlotType_<ContainerAllocator>  _type_type;
  _type_type type;

   typedef uint8_t _occupied_type;
  _occupied_type occupied;

   typedef double _width_type;
  _width_type width;

   typedef double _length_type;
  _length_type length;





  typedef boost::shared_ptr< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> const> ConstPtr;

}; // struct ParkingSlot_

typedef ::mviz_apa_show::ParkingSlot_<std::allocator<void> > ParkingSlot;

typedef boost::shared_ptr< ::mviz_apa_show::ParkingSlot > ParkingSlotPtr;
typedef boost::shared_ptr< ::mviz_apa_show::ParkingSlot const> ParkingSlotConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::ParkingSlot_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::ParkingSlot_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ParkingSlot_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.corner_pts == rhs.corner_pts &&
    lhs.type == rhs.type &&
    lhs.occupied == rhs.occupied &&
    lhs.width == rhs.width &&
    lhs.length == rhs.length;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::ParkingSlot_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ParkingSlot_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3e1ed7bda9b72a4d29ae5e0cdbdcb007";
  }

  static const char* value(const ::mviz_apa_show::ParkingSlot_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3e1ed7bda9b72a4dULL;
  static const uint64_t static_value2 = 0x29ae5e0cdbdcb007ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/ParkingSlot";
  }

  static const char* value(const ::mviz_apa_show::ParkingSlot_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 id\n"
"CornerPt[] corner_pts    # 车位四个角点坐标\n"
"ParkSlotType type               # 车位类型\n"
"bool occupied                   # 车位里是否有障碍物\n"
"float64 width                     # 车位宽（m）\n"
"float64 length                    # 车位长 (m)\n"
"================================================================================\n"
"MSG: mviz_apa_show/CornerPt\n"
"uint32 id \n"
"Point2f image_pt   # 图像坐标 (pixel)\n"
"Point2f vehicle_pt   # 相对车身的物理坐标（m）(后处理模块debug用，右前上)\n"
"Point2f odom_pt   # odom坐标系下的物理坐标（m）\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
"================================================================================\n"
"MSG: mviz_apa_show/ParkSlotType\n"
"ParkSlotTypeEnum type\n"
"================================================================================\n"
"MSG: mviz_apa_show/ParkSlotTypeEnum\n"
"int8 kUnknown = 0 \n"
"int8 kVerticalSpot = 1           # 垂直车位\n"
"int8 kHorizontalSpot = 2         # 水平车位\n"
"int8 kObliqueSpot = 3            # 斜车位\n"
"int8 value\n"
;
  }

  static const char* value(const ::mviz_apa_show::ParkingSlot_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.corner_pts);
      stream.next(m.type);
      stream.next(m.occupied);
      stream.next(m.width);
      stream.next(m.length);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ParkingSlot_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::ParkingSlot_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::ParkingSlot_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "corner_pts[]" << std::endl;
    for (size_t i = 0; i < v.corner_pts.size(); ++i)
    {
      s << indent << "  corner_pts[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::CornerPt_<ContainerAllocator> >::stream(s, indent + "    ", v.corner_pts[i]);
    }
    s << indent << "type: ";
    s << std::endl;
    Printer< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
    s << indent << "occupied: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.occupied);
    s << indent << "width: ";
    Printer<double>::stream(s, indent + "  ", v.width);
    s << indent << "length: ";
    Printer<double>::stream(s, indent + "  ", v.length);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_PARKINGSLOT_H