// Generated by gencpp from file mviz_apa_show/MeasureType.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_MEASURETYPE_H
#define MVIZ_APA_SHOW_MESSAGE_MEASURETYPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/MeasureTypeEnum.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct MeasureType_
{
  typedef MeasureType_<ContainerAllocator> Type;

  MeasureType_()
    : type()  {
    }
  MeasureType_(const ContainerAllocator& _alloc)
    : type(_alloc)  {
  (void)_alloc;
    }



   typedef  ::mviz_apa_show::MeasureTypeEnum_<ContainerAllocator>  _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::mviz_apa_show::MeasureType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::MeasureType_<ContainerAllocator> const> ConstPtr;

}; // struct MeasureType_

typedef ::mviz_apa_show::MeasureType_<std::allocator<void> > MeasureType;

typedef boost::shared_ptr< ::mviz_apa_show::MeasureType > MeasureTypePtr;
typedef boost::shared_ptr< ::mviz_apa_show::MeasureType const> MeasureTypeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::MeasureType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::MeasureType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::MeasureType_<ContainerAllocator1> & lhs, const ::mviz_apa_show::MeasureType_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::MeasureType_<ContainerAllocator1> & lhs, const ::mviz_apa_show::MeasureType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::MeasureType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::MeasureType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::MeasureType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb70fde2b440fdb7b7644836096fbeb5";
  }

  static const char* value(const ::mviz_apa_show::MeasureType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb70fde2b440fdb7ULL;
  static const uint64_t static_value2 = 0xb7644836096fbeb5ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/MeasureType";
  }

  static const char* value(const ::mviz_apa_show::MeasureType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MeasureTypeEnum type\n"
"================================================================================\n"
"MSG: mviz_apa_show/MeasureTypeEnum\n"
"int8 kVehicleMeasureHeadReg = 0\n"
"int8 kVehicleMeasureTailReg = 1\n"
"int8 kVehicleMeasureDetect = 2\n"
"int8 kVehicleMeasureWheel = 3\n"
"int8 kVehicleMeasurePlate = 4\n"
"int8 value\n"
;
  }

  static const char* value(const ::mviz_apa_show::MeasureType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MeasureType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::MeasureType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::MeasureType_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    s << std::endl;
    Printer< ::mviz_apa_show::MeasureTypeEnum_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_MEASURETYPE_H
