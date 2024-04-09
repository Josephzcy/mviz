// Generated by gencpp from file mviz_apa_show/FreespacePoints.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_FREESPACEPOINTS_H
#define MVIZ_APA_SHOW_MESSAGE_FREESPACEPOINTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Point2fList.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct FreespacePoints_
{
  typedef FreespacePoints_<ContainerAllocator> Type;

  FreespacePoints_()
    : timestamp(0)
    , frame_id(0)
    , tick(0)
    , freespacepoints()  {
    }
  FreespacePoints_(const ContainerAllocator& _alloc)
    : timestamp(0)
    , frame_id(0)
    , tick(0)
    , freespacepoints(_alloc)  {
  (void)_alloc;
    }



   typedef uint64_t _timestamp_type;
  _timestamp_type timestamp;

   typedef uint64_t _frame_id_type;
  _frame_id_type frame_id;

   typedef uint64_t _tick_type;
  _tick_type tick;

   typedef std::vector< ::mviz_apa_show::Point2fList_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::Point2fList_<ContainerAllocator> >::other >  _freespacepoints_type;
  _freespacepoints_type freespacepoints;





  typedef boost::shared_ptr< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> const> ConstPtr;

}; // struct FreespacePoints_

typedef ::mviz_apa_show::FreespacePoints_<std::allocator<void> > FreespacePoints;

typedef boost::shared_ptr< ::mviz_apa_show::FreespacePoints > FreespacePointsPtr;
typedef boost::shared_ptr< ::mviz_apa_show::FreespacePoints const> FreespacePointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::FreespacePoints_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::FreespacePoints_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreespacePoints_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.frame_id == rhs.frame_id &&
    lhs.tick == rhs.tick &&
    lhs.freespacepoints == rhs.freespacepoints;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::FreespacePoints_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreespacePoints_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb8005ed713cb97562aa0cfaae23d36d";
  }

  static const char* value(const ::mviz_apa_show::FreespacePoints_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb8005ed713cb975ULL;
  static const uint64_t static_value2 = 0x62aa0cfaae23d36dULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/FreespacePoints";
  }

  static const char* value(const ::mviz_apa_show::FreespacePoints_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "  uint64 timestamp\n"
"  uint64 frame_id\n"
"  uint64 tick\n"
"  Point2fList[] freespacepoints\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2fList\n"
"Point2f[] points\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::mviz_apa_show::FreespacePoints_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.frame_id);
      stream.next(m.tick);
      stream.next(m.freespacepoints);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FreespacePoints_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::FreespacePoints_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::FreespacePoints_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "frame_id: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.frame_id);
    s << indent << "tick: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.tick);
    s << indent << "freespacepoints[]" << std::endl;
    for (size_t i = 0; i < v.freespacepoints.size(); ++i)
    {
      s << indent << "  freespacepoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::Point2fList_<ContainerAllocator> >::stream(s, indent + "    ", v.freespacepoints[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_FREESPACEPOINTS_H
