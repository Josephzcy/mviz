// Generated by gencpp from file mviz_apa_show/CurveCoeff.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_CURVECOEFF_H
#define MVIZ_APA_SHOW_MESSAGE_CURVECOEFF_H


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
struct CurveCoeff_
{
  typedef CurveCoeff_<ContainerAllocator> Type;

  CurveCoeff_()
    : longitude_min(0.0)
    , longitude_max(0.0)
    , c0(0.0)
    , c1(0.0)
    , c2(0.0)
    , c3(0.0)
    , dev_c0(0.0)
    , dev_c1(0.0)
    , dev_c2(0.0)
    , dev_c3(0.0)  {
    }
  CurveCoeff_(const ContainerAllocator& _alloc)
    : longitude_min(0.0)
    , longitude_max(0.0)
    , c0(0.0)
    , c1(0.0)
    , c2(0.0)
    , c3(0.0)
    , dev_c0(0.0)
    , dev_c1(0.0)
    , dev_c2(0.0)
    , dev_c3(0.0)  {
  (void)_alloc;
    }



   typedef float _longitude_min_type;
  _longitude_min_type longitude_min;

   typedef float _longitude_max_type;
  _longitude_max_type longitude_max;

   typedef double _c0_type;
  _c0_type c0;

   typedef double _c1_type;
  _c1_type c1;

   typedef double _c2_type;
  _c2_type c2;

   typedef double _c3_type;
  _c3_type c3;

   typedef double _dev_c0_type;
  _dev_c0_type dev_c0;

   typedef double _dev_c1_type;
  _dev_c1_type dev_c1;

   typedef double _dev_c2_type;
  _dev_c2_type dev_c2;

   typedef double _dev_c3_type;
  _dev_c3_type dev_c3;





  typedef boost::shared_ptr< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> const> ConstPtr;

}; // struct CurveCoeff_

typedef ::mviz_apa_show::CurveCoeff_<std::allocator<void> > CurveCoeff;

typedef boost::shared_ptr< ::mviz_apa_show::CurveCoeff > CurveCoeffPtr;
typedef boost::shared_ptr< ::mviz_apa_show::CurveCoeff const> CurveCoeffConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::CurveCoeff_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::CurveCoeff_<ContainerAllocator1> & lhs, const ::mviz_apa_show::CurveCoeff_<ContainerAllocator2> & rhs)
{
  return lhs.longitude_min == rhs.longitude_min &&
    lhs.longitude_max == rhs.longitude_max &&
    lhs.c0 == rhs.c0 &&
    lhs.c1 == rhs.c1 &&
    lhs.c2 == rhs.c2 &&
    lhs.c3 == rhs.c3 &&
    lhs.dev_c0 == rhs.dev_c0 &&
    lhs.dev_c1 == rhs.dev_c1 &&
    lhs.dev_c2 == rhs.dev_c2 &&
    lhs.dev_c3 == rhs.dev_c3;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::CurveCoeff_<ContainerAllocator1> & lhs, const ::mviz_apa_show::CurveCoeff_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e008e6369625abb2abe471a66bb722c4";
  }

  static const char* value(const ::mviz_apa_show::CurveCoeff_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe008e6369625abb2ULL;
  static const uint64_t static_value2 = 0xabe471a66bb722c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/CurveCoeff";
  }

  static const char* value(const ::mviz_apa_show::CurveCoeff_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 曲线方程: y = c0 + c1*x + c2*x^2 + c3*x^3\n"
"float32 longitude_min  # view range start 类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)\n"
"float32 longitude_max  # view range end   类型:float 字节数:4 取值范围:(FLT_MIN~FLT_MAX)\n"
"float64 c0            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)\n"
"float64 c1            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)\n"
"float64 c2            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)\n"
"float64 c3            # 类型:float64 字节数:8 取值范围:(DOUBLE_MIN~DOUBLE_MAX)\n"
"float64 dev_c0        # c0的标准差，即，车道线横向位置的标准差\n"
"float64 dev_c1		  # c1的标准差，即，车道线航向角的标准差\n"
"float64 dev_c2        # c2的标准差，即，车道线原点处曲率的标准差\n"
"float64 dev_c3       # c3的标准差，即，车道线原点处曲率变化率的标注差\n"
;
  }

  static const char* value(const ::mviz_apa_show::CurveCoeff_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.longitude_min);
      stream.next(m.longitude_max);
      stream.next(m.c0);
      stream.next(m.c1);
      stream.next(m.c2);
      stream.next(m.c3);
      stream.next(m.dev_c0);
      stream.next(m.dev_c1);
      stream.next(m.dev_c2);
      stream.next(m.dev_c3);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CurveCoeff_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::CurveCoeff_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::CurveCoeff_<ContainerAllocator>& v)
  {
    s << indent << "longitude_min: ";
    Printer<float>::stream(s, indent + "  ", v.longitude_min);
    s << indent << "longitude_max: ";
    Printer<float>::stream(s, indent + "  ", v.longitude_max);
    s << indent << "c0: ";
    Printer<double>::stream(s, indent + "  ", v.c0);
    s << indent << "c1: ";
    Printer<double>::stream(s, indent + "  ", v.c1);
    s << indent << "c2: ";
    Printer<double>::stream(s, indent + "  ", v.c2);
    s << indent << "c3: ";
    Printer<double>::stream(s, indent + "  ", v.c3);
    s << indent << "dev_c0: ";
    Printer<double>::stream(s, indent + "  ", v.dev_c0);
    s << indent << "dev_c1: ";
    Printer<double>::stream(s, indent + "  ", v.dev_c1);
    s << indent << "dev_c2: ";
    Printer<double>::stream(s, indent + "  ", v.dev_c2);
    s << indent << "dev_c3: ";
    Printer<double>::stream(s, indent + "  ", v.dev_c3);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_CURVECOEFF_H