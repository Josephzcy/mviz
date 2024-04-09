// Generated by gencpp from file mviz_apa_show/XYZ.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_XYZ_H
#define MVIZ_APA_SHOW_MESSAGE_XYZ_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/EvalStats.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct XYZ_
{
  typedef XYZ_<ContainerAllocator> Type;

  XYZ_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , evals()  {
    }
  XYZ_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , evals(_alloc)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef std::vector< ::mviz_apa_show::EvalStats_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::EvalStats_<ContainerAllocator> >::other >  _evals_type;
  _evals_type evals;





  typedef boost::shared_ptr< ::mviz_apa_show::XYZ_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::XYZ_<ContainerAllocator> const> ConstPtr;

}; // struct XYZ_

typedef ::mviz_apa_show::XYZ_<std::allocator<void> > XYZ;

typedef boost::shared_ptr< ::mviz_apa_show::XYZ > XYZPtr;
typedef boost::shared_ptr< ::mviz_apa_show::XYZ const> XYZConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::XYZ_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::XYZ_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::XYZ_<ContainerAllocator1> & lhs, const ::mviz_apa_show::XYZ_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.evals == rhs.evals;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::XYZ_<ContainerAllocator1> & lhs, const ::mviz_apa_show::XYZ_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::XYZ_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::XYZ_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::XYZ_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::XYZ_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::XYZ_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::XYZ_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::XYZ_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff700a66018c16e43728083e54bed8d7";
  }

  static const char* value(const ::mviz_apa_show::XYZ_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff700a66018c16e4ULL;
  static const uint64_t static_value2 = 0x3728083e54bed8d7ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::XYZ_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/XYZ";
  }

  static const char* value(const ::mviz_apa_show::XYZ_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::XYZ_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 z\n"
"EvalStats[] evals\n"
"================================================================================\n"
"MSG: mviz_apa_show/EvalStats\n"
"float32 mean\n"
"float32 std \n"
"float32 prob\n"
;
  }

  static const char* value(const ::mviz_apa_show::XYZ_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::XYZ_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.evals);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct XYZ_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::XYZ_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::XYZ_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "evals[]" << std::endl;
    for (size_t i = 0; i < v.evals.size(); ++i)
    {
      s << indent << "  evals[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::EvalStats_<ContainerAllocator> >::stream(s, indent + "    ", v.evals[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_XYZ_H