// Generated by gencpp from file mviz_apa_show/Pole.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_POLE_H
#define MVIZ_APA_SHOW_MESSAGE_POLE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Point2fList.h>
#include <mviz_apa_show/Point3DList.h>
#include <geometry_msgs/Point.h>
#include <mviz_apa_show/Point2f.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct Pole_
{
  typedef Pole_<ContainerAllocator> Type;

  Pole_()
    : id(0)
    , corner_pt_image_coord()
    , corner_pt_vehicle_coord()
    , centroid_vehicle_coord()
    , confidence(0.0)
    , centroid_image_coord()
    , pos(0)
    , camera_id(0)  {
    }
  Pole_(const ContainerAllocator& _alloc)
    : id(0)
    , corner_pt_image_coord(_alloc)
    , corner_pt_vehicle_coord(_alloc)
    , centroid_vehicle_coord(_alloc)
    , confidence(0.0)
    , centroid_image_coord(_alloc)
    , pos(0)
    , camera_id(0)  {
  (void)_alloc;
    }



   typedef uint32_t _id_type;
  _id_type id;

   typedef  ::mviz_apa_show::Point2fList_<ContainerAllocator>  _corner_pt_image_coord_type;
  _corner_pt_image_coord_type corner_pt_image_coord;

   typedef  ::mviz_apa_show::Point3DList_<ContainerAllocator>  _corner_pt_vehicle_coord_type;
  _corner_pt_vehicle_coord_type corner_pt_vehicle_coord;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _centroid_vehicle_coord_type;
  _centroid_vehicle_coord_type centroid_vehicle_coord;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef  ::mviz_apa_show::Point2f_<ContainerAllocator>  _centroid_image_coord_type;
  _centroid_image_coord_type centroid_image_coord;

   typedef int32_t _pos_type;
  _pos_type pos;

   typedef uint32_t _camera_id_type;
  _camera_id_type camera_id;





  typedef boost::shared_ptr< ::mviz_apa_show::Pole_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::Pole_<ContainerAllocator> const> ConstPtr;

}; // struct Pole_

typedef ::mviz_apa_show::Pole_<std::allocator<void> > Pole;

typedef boost::shared_ptr< ::mviz_apa_show::Pole > PolePtr;
typedef boost::shared_ptr< ::mviz_apa_show::Pole const> PoleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::Pole_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::Pole_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::Pole_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Pole_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.corner_pt_image_coord == rhs.corner_pt_image_coord &&
    lhs.corner_pt_vehicle_coord == rhs.corner_pt_vehicle_coord &&
    lhs.centroid_vehicle_coord == rhs.centroid_vehicle_coord &&
    lhs.confidence == rhs.confidence &&
    lhs.centroid_image_coord == rhs.centroid_image_coord &&
    lhs.pos == rhs.pos &&
    lhs.camera_id == rhs.camera_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::Pole_<ContainerAllocator1> & lhs, const ::mviz_apa_show::Pole_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Pole_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::Pole_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Pole_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::Pole_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Pole_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::Pole_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::Pole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a53dc99536b6accd184351e8a168e463";
  }

  static const char* value(const ::mviz_apa_show::Pole_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa53dc99536b6accdULL;
  static const uint64_t static_value2 = 0x184351e8a168e463ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::Pole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/Pole";
  }

  static const char* value(const ::mviz_apa_show::Pole_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::Pole_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 id                             # id 类型:uint32 字节数:4 取值范围:(0~2^32)\n"
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

  static const char* value(const ::mviz_apa_show::Pole_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::Pole_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.corner_pt_image_coord);
      stream.next(m.corner_pt_vehicle_coord);
      stream.next(m.centroid_vehicle_coord);
      stream.next(m.confidence);
      stream.next(m.centroid_image_coord);
      stream.next(m.pos);
      stream.next(m.camera_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pole_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::Pole_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::Pole_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "corner_pt_image_coord: ";
    s << std::endl;
    Printer< ::mviz_apa_show::Point2fList_<ContainerAllocator> >::stream(s, indent + "  ", v.corner_pt_image_coord);
    s << indent << "corner_pt_vehicle_coord: ";
    s << std::endl;
    Printer< ::mviz_apa_show::Point3DList_<ContainerAllocator> >::stream(s, indent + "  ", v.corner_pt_vehicle_coord);
    s << indent << "centroid_vehicle_coord: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.centroid_vehicle_coord);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "centroid_image_coord: ";
    s << std::endl;
    Printer< ::mviz_apa_show::Point2f_<ContainerAllocator> >::stream(s, indent + "  ", v.centroid_image_coord);
    s << indent << "pos: ";
    Printer<int32_t>::stream(s, indent + "  ", v.pos);
    s << indent << "camera_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.camera_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_POLE_H
