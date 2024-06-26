// Generated by gencpp from file mviz_apa_show/Angle3f.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_ANGLE3F_H
#define MVIZ_APA_SHOW_MESSAGE_ANGLE3F_H


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
struct Angle3f_
{
  typedef Angle3f_<ContainerAllocator> Type;

  Angle3f_()
    : center(0.0)
    , left(0.0)
    , right(0.0)  {
    }
  Angle3f_(const ContainerAllocator& _alloc)
    : center(0.0)
    , left(0.0)
    , right(0.0)  {
  (void)_alloc;
    }



   typedef float _center_type;
  _center_type center;

   typedef float _left_type;
  _left_type left;

   typedef float _right_type;
  _right_type right;





  typedef boost::shared_ptr< ::mviz_apa_show::Angle3f_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::Angle3f_<ContainerAllocator> const> ConstPtr;

}; // struct Angle3f_

typedef ::mviz_apa_show::Angle3f_<std::allocator<void> > Angle3f;

typedef boost::shared_ptr< ::mviz_apa_show::Angle3f > Angle3fPtr;
typedef boost::shared_ptr< ::mviz_apa_show::Angle3f const> Angle3fConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::Angle3f_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::Angle3f_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::Angle3f_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Angle3f_<ContainerAllocator2> & rhs)
{
  return lhs.center == rhs.center &&
    lhs.left == rhs.left &&
    lhs.right == rhs.right;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::Angle3f_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Angle3f_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Angle3f_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Angle3f_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Angle3f_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bee0ec9c390279e4703776d47eb80967";
  }

  static const char* value(const ::mviz_apa_show::Angle3f_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbee0ec9c390279e4ULL;
  static const uint64_t static_value2 = 0x703776d47eb80967ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/Angle3f";
  }

  static const char* value(const ::mviz_apa_show::Angle3f_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 center\n"
"float32 left \n"
"float32 right \n"
;
  }

  static const char* value(const ::mviz_apa_show::Angle3f_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.center);
      stream.next(m.left);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Angle3f_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::Angle3f_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::Angle3f_<ContainerAllocator>& v)
  {
    s << indent << "center: ";
    Printer<float>::stream(s, indent + "  ", v.center);
    s << indent << "left: ";
    Printer<float>::stream(s, indent + "  ", v.left);
    s << indent << "right: ";
    Printer<float>::stream(s, indent + "  ", v.right);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_ANGLE3F_H
