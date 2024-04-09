// Generated by gencpp from file mviz_apa_show/PoleList.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_POLELIST_H
#define MVIZ_APA_SHOW_MESSAGE_POLELIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Pole.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct PoleList_
{
  typedef PoleList_<ContainerAllocator> Type;

  PoleList_()
    : poles()  {
    }
  PoleList_(const ContainerAllocator& _alloc)
    : poles(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::mviz_apa_show::Pole_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::Pole_<ContainerAllocator> >::other >  _poles_type;
  _poles_type poles;





  typedef boost::shared_ptr< ::mviz_apa_show::PoleList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::PoleList_<ContainerAllocator> const> ConstPtr;

}; // struct PoleList_

typedef ::mviz_apa_show::PoleList_<std::allocator<void> > PoleList;

typedef boost::shared_ptr< ::mviz_apa_show::PoleList > PoleListPtr;
typedef boost::shared_ptr< ::mviz_apa_show::PoleList const> PoleListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::PoleList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::PoleList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::PoleList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::PoleList_<ContainerAllocator2> & rhs)
{
  return lhs.poles == rhs.poles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::PoleList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::PoleList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::PoleList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::PoleList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::PoleList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::PoleList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::PoleList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::PoleList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::PoleList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a860b4617f6b75907f5b4aab8a08d69e";
  }

  static const char* value(const ::mviz_apa_show::PoleList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa860b4617f6b7590ULL;
  static const uint64_t static_value2 = 0x7f5b4aab8a08d69eULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::PoleList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/PoleList";
  }

  static const char* value(const ::mviz_apa_show::PoleList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::PoleList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Pole[] poles\n"
"================================================================================\n"
"MSG: mviz_apa_show/Pole\n"
"uint32 id                             # id 类型:uint32 字节数:4 取值范围:(0~2^32)\n"
"Point2fList corner_pt_image_coord     # 关键点图像坐标 字节数:64 取值范围(w>=x>=0, h>=y>=0) 容量:4\n"
"Point3DList corner_pt_vehicle_coord   # 关键点世界坐标 字节数:96 取值范围(100>=x>=0, 20>=y>=-20) 容量:4\n"
"geometry_msgs/Point centroid_vehicle_coord        # 质心世界坐标 字节数:24 取值范围(100>=x>=0, 20>=y>=-20)\n"
"float32 confidence                    # 置信度 类型:float 字节数:4 取值范围:(0~1)\n"
"Point2f centroid_image_coord\n"
"int32 pos                             # 位置 1->左边 2->右边\n"
"uint32 camera_id                      # 摄像头id\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2fList\n"
"Point2f[] points\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point3DList\n"
"geometry_msgs/Point[] points \n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::mviz_apa_show::PoleList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::PoleList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.poles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoleList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::PoleList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::PoleList_<ContainerAllocator>& v)
  {
    s << indent << "poles[]" << std::endl;
    for (size_t i = 0; i < v.poles.size(); ++i)
    {
      s << indent << "  poles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::Pole_<ContainerAllocator> >::stream(s, indent + "    ", v.poles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_POLELIST_H