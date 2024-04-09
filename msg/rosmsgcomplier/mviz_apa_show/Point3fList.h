// Generated by gencpp from file mviz_apa_show/Point3fList.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_POINT3FLIST_H
#define MVIZ_APA_SHOW_MESSAGE_POINT3FLIST_H


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
struct Point3fList_
{
  typedef Point3fList_<ContainerAllocator> Type;

  Point3fList_()
    : points()  {
    }
  Point3fList_(const ContainerAllocator& _alloc)
    : points(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::mviz_apa_show::Point3f_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::Point3f_<ContainerAllocator> >::other >  _points_type;
  _points_type points;





  typedef boost::shared_ptr< ::mviz_apa_show::Point3fList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::Point3fList_<ContainerAllocator> const> ConstPtr;

}; // struct Point3fList_

typedef ::mviz_apa_show::Point3fList_<std::allocator<void> > Point3fList;

typedef boost::shared_ptr< ::mviz_apa_show::Point3fList > Point3fListPtr;
typedef boost::shared_ptr< ::mviz_apa_show::Point3fList const> Point3fListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::Point3fList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::Point3fList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::Point3fList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Point3fList_<ContainerAllocator2> & rhs)
{
  return lhs.points == rhs.points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::Point3fList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Point3fList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Point3fList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Point3fList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Point3fList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cd60a26494a087f577976f0329fa120e";
  }

  static const char* value(const ::mviz_apa_show::Point3fList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcd60a26494a087f5ULL;
  static const uint64_t static_value2 = 0x77976f0329fa120eULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/Point3fList";
  }

  static const char* value(const ::mviz_apa_show::Point3fList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Point3f[] points\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point3f\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::mviz_apa_show::Point3fList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Point3fList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::Point3fList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::Point3fList_<ContainerAllocator>& v)
  {
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::Point3f_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_POINT3FLIST_H
