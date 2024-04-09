// Generated by gencpp from file mviz_apa_show/PlanningPose.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_PLANNINGPOSE_H
#define MVIZ_APA_SHOW_MESSAGE_PLANNINGPOSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mviz_apa_show
{
template <class ContainerAllocator>
struct PlanningPose_
{
  typedef PlanningPose_<ContainerAllocator> Type;

  PlanningPose_()
    : timestamp(0)
    , tick(0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  PlanningPose_(const ContainerAllocator& _alloc)
    : timestamp(0)
    , tick(0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef uint64_t _timestamp_type;
  _timestamp_type timestamp;

   typedef uint64_t _tick_type;
  _tick_type tick;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::mviz_apa_show::PlanningPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::PlanningPose_<ContainerAllocator> const> ConstPtr;

}; // struct PlanningPose_

typedef ::mviz_apa_show::PlanningPose_<std::allocator<void> > PlanningPose;

typedef boost::shared_ptr< ::mviz_apa_show::PlanningPose > PlanningPosePtr;
typedef boost::shared_ptr< ::mviz_apa_show::PlanningPose const> PlanningPoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::PlanningPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::PlanningPose_<ContainerAllocator1> & lhs, const ::mviz_apa_show::PlanningPose_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.tick == rhs.tick &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::PlanningPose_<ContainerAllocator1> & lhs, const ::mviz_apa_show::PlanningPose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::PlanningPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::PlanningPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::PlanningPose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f2c6af2b7e5b730eee00b753ddd251e";
  }

  static const char* value(const ::mviz_apa_show::PlanningPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f2c6af2b7e5b730ULL;
  static const uint64_t static_value2 = 0xeee00b753ddd251eULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/PlanningPose";
  }

  static const char* value(const ::mviz_apa_show::PlanningPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 timestamp\n"
"uint64 tick\n"
"float32 x\n"
"float32 y\n"
"float32 theta\n"
;
  }

  static const char* value(const ::mviz_apa_show::PlanningPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.tick);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlanningPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::PlanningPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::PlanningPose_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "tick: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.tick);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_PLANNINGPOSE_H