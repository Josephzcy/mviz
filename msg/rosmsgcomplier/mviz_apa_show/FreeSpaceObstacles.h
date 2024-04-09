// Generated by gencpp from file mviz_apa_show/FreeSpaceObstacles.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_FREESPACEOBSTACLES_H
#define MVIZ_APA_SHOW_MESSAGE_FREESPACEOBSTACLES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/FreespaceObstacle.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct FreeSpaceObstacles_
{
  typedef FreeSpaceObstacles_<ContainerAllocator> Type;

  FreeSpaceObstacles_()
    : timestamp(0)
    , obstacles()  {
    }
  FreeSpaceObstacles_(const ContainerAllocator& _alloc)
    : timestamp(0)
    , obstacles(_alloc)  {
  (void)_alloc;
    }



   typedef uint64_t _timestamp_type;
  _timestamp_type timestamp;

   typedef std::vector< ::mviz_apa_show::FreespaceObstacle_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::FreespaceObstacle_<ContainerAllocator> >::other >  _obstacles_type;
  _obstacles_type obstacles;





  typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> const> ConstPtr;

}; // struct FreeSpaceObstacles_

typedef ::mviz_apa_show::FreeSpaceObstacles_<std::allocator<void> > FreeSpaceObstacles;

typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaceObstacles > FreeSpaceObstaclesPtr;
typedef boost::shared_ptr< ::mviz_apa_show::FreeSpaceObstacles const> FreeSpaceObstaclesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.obstacles == rhs.obstacles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator1> & lhs, const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "764482b85a9f69b0ddb6462e100efd64";
  }

  static const char* value(const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x764482b85a9f69b0ULL;
  static const uint64_t static_value2 = 0xddb6462e100efd64ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/FreeSpaceObstacles";
  }

  static const char* value(const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 timestamp                      # 时戳, milliseconds\n"
"FreespaceObstacle[] obstacles\n"
"================================================================================\n"
"MSG: mviz_apa_show/FreespaceObstacle\n"
"uint32 id\n"
"Point2f[] points\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.obstacles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FreeSpaceObstacles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::FreeSpaceObstacles_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "obstacles[]" << std::endl;
    for (size_t i = 0; i < v.obstacles.size(); ++i)
    {
      s << indent << "  obstacles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::FreespaceObstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_FREESPACEOBSTACLES_H
