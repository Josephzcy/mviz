// Generated by gencpp from file mviz_apa_show/Pose.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_POSE_H
#define MVIZ_APA_SHOW_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Point3f.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : timestamp_us(0)
    , pose()  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : timestamp_us(0)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef uint64_t _timestamp_us_type;
  _timestamp_us_type timestamp_us;

   typedef  ::mviz_apa_show::Point3f_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::mviz_apa_show::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::mviz_apa_show::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::mviz_apa_show::Pose > PosePtr;
typedef boost::shared_ptr< ::mviz_apa_show::Pose const> PoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::Pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::Pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::Pose_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Pose_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp_us == rhs.timestamp_us &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::Pose_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Pose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2db30c33c3a9e26fc73f831279973814";
  }

  static const char* value(const ::mviz_apa_show::Pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2db30c33c3a9e26fULL;
  static const uint64_t static_value2 = 0xc73f831279973814ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/Pose";
  }

  static const char* value(const ::mviz_apa_show::Pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 timestamp_us    # time stamp us\n"
"Point3f pose           #  [x, y, theta] [meter, meter, rad]\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point3f\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::mviz_apa_show::Pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::Pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp_us);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::Pose_<ContainerAllocator>& v)
  {
    s << indent << "timestamp_us: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp_us);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::mviz_apa_show::Point3f_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_POSE_H
