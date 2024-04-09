// Generated by gencpp from file mviz_apa_show/ProjectionModelEnum.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_PROJECTIONMODELENUM_H
#define MVIZ_APA_SHOW_MESSAGE_PROJECTIONMODELENUM_H


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
struct ProjectionModelEnum_
{
  typedef ProjectionModelEnum_<ContainerAllocator> Type;

  ProjectionModelEnum_()
    : value(0)  {
    }
  ProjectionModelEnum_(const ContainerAllocator& _alloc)
    : value(0)  {
  (void)_alloc;
    }



   typedef int8_t _value_type;
  _value_type value;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(PRJ_MODEL_UNKNOWN)
  #undef PRJ_MODEL_UNKNOWN
#endif
#if defined(_WIN32) && defined(FISHEYE)
  #undef FISHEYE
#endif
#if defined(_WIN32) && defined(MEI)
  #undef MEI
#endif
#if defined(_WIN32) && defined(PIN_HOLE)
  #undef PIN_HOLE
#endif
#if defined(_WIN32) && defined(ATAN)
  #undef ATAN
#endif
#if defined(_WIN32) && defined(DAVIDE_SCARAMUZZA)
  #undef DAVIDE_SCARAMUZZA
#endif

  enum {
    PRJ_MODEL_UNKNOWN = 0,
    FISHEYE = 1,
    MEI = 2,
    PIN_HOLE = 3,
    ATAN = 4,
    DAVIDE_SCARAMUZZA = 5,
  };


  typedef boost::shared_ptr< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> const> ConstPtr;

}; // struct ProjectionModelEnum_

typedef ::mviz_apa_show::ProjectionModelEnum_<std::allocator<void> > ProjectionModelEnum;

typedef boost::shared_ptr< ::mviz_apa_show::ProjectionModelEnum > ProjectionModelEnumPtr;
typedef boost::shared_ptr< ::mviz_apa_show::ProjectionModelEnum const> ProjectionModelEnumConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator2> & rhs)
{
  return lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6404b3626be7ab2b32e6930befeb0f11";
  }

  static const char* value(const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6404b3626be7ab2bULL;
  static const uint64_t static_value2 = 0x32e6930befeb0f11ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/ProjectionModelEnum";
  }

  static const char* value(const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 PRJ_MODEL_UNKNOWN = 0\n"
"int8 FISHEYE = 1\n"
"int8 MEI = 2\n"
"int8 PIN_HOLE = 3\n"
"int8 ATAN = 4\n"
"int8 DAVIDE_SCARAMUZZA = 5\n"
"int8 value\n"
;
  }

  static const char* value(const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ProjectionModelEnum_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::ProjectionModelEnum_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<int8_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_PROJECTIONMODELENUM_H
