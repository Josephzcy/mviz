// Generated by gencpp from file mviz_apa_show/ParkSlotType.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_PARKSLOTTYPE_H
#define MVIZ_APA_SHOW_MESSAGE_PARKSLOTTYPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/ParkSlotTypeEnum.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct ParkSlotType_
{
  typedef ParkSlotType_<ContainerAllocator> Type;

  ParkSlotType_()
    : type()  {
    }
  ParkSlotType_(const ContainerAllocator& _alloc)
    : type(_alloc)  {
  (void)_alloc;
    }



   typedef  ::mviz_apa_show::ParkSlotTypeEnum_<ContainerAllocator>  _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> const> ConstPtr;

}; // struct ParkSlotType_

typedef ::mviz_apa_show::ParkSlotType_<std::allocator<void> > ParkSlotType;

typedef boost::shared_ptr< ::mviz_apa_show::ParkSlotType > ParkSlotTypePtr;
typedef boost::shared_ptr< ::mviz_apa_show::ParkSlotType const> ParkSlotTypeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::ParkSlotType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::ParkSlotType_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ParkSlotType_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::ParkSlotType_<ContainerAllocator1> & lhs, const ::mviz_apa_show::ParkSlotType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a7f954bd1c1cbfa6525a3417539a4b0b";
  }

  static const char* value(const ::mviz_apa_show::ParkSlotType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa7f954bd1c1cbfa6ULL;
  static const uint64_t static_value2 = 0x525a3417539a4b0bULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/ParkSlotType";
  }

  static const char* value(const ::mviz_apa_show::ParkSlotType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ParkSlotTypeEnum type\n"
"================================================================================\n"
"MSG: mviz_apa_show/ParkSlotTypeEnum\n"
"int8 kUnknown = 0 \n"
"int8 kVerticalSpot = 1           # 垂直车位\n"
"int8 kHorizontalSpot = 2         # 水平车位\n"
"int8 kObliqueSpot = 3            # 斜车位\n"
"int8 value\n"
;
  }

  static const char* value(const ::mviz_apa_show::ParkSlotType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ParkSlotType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::ParkSlotType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::ParkSlotType_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    s << std::endl;
    Printer< ::mviz_apa_show::ParkSlotTypeEnum_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_PARKSLOTTYPE_H
