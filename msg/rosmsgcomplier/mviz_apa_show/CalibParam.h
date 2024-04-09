// Generated by gencpp from file mviz_apa_show/CalibParam.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_CALIBPARAM_H
#define MVIZ_APA_SHOW_MESSAGE_CALIBPARAM_H


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
struct CalibParam_
{
  typedef CalibParam_<ContainerAllocator> Type;

  CalibParam_()
    : camera_pitch(0.0)
    , camera_yaw(0.0)
    , camera_roll(0.0)
    , camera_cu(0.0)
    , camera_cv(0.0)
    , camera_fov_h(0.0)
    , camera_fov_w(0.0)
    , camera_height(0.0)
    , left_dist_to_camera(0.0)
    , right_dist_to_camera(0.0)
    , front_dist_to_camera(0.0)
    , front_wheel_camera_dist(0.0)
    , camera_id(0)  {
    }
  CalibParam_(const ContainerAllocator& _alloc)
    : camera_pitch(0.0)
    , camera_yaw(0.0)
    , camera_roll(0.0)
    , camera_cu(0.0)
    , camera_cv(0.0)
    , camera_fov_h(0.0)
    , camera_fov_w(0.0)
    , camera_height(0.0)
    , left_dist_to_camera(0.0)
    , right_dist_to_camera(0.0)
    , front_dist_to_camera(0.0)
    , front_wheel_camera_dist(0.0)
    , camera_id(0)  {
  (void)_alloc;
    }



   typedef double _camera_pitch_type;
  _camera_pitch_type camera_pitch;

   typedef double _camera_yaw_type;
  _camera_yaw_type camera_yaw;

   typedef double _camera_roll_type;
  _camera_roll_type camera_roll;

   typedef double _camera_cu_type;
  _camera_cu_type camera_cu;

   typedef double _camera_cv_type;
  _camera_cv_type camera_cv;

   typedef double _camera_fov_h_type;
  _camera_fov_h_type camera_fov_h;

   typedef double _camera_fov_w_type;
  _camera_fov_w_type camera_fov_w;

   typedef double _camera_height_type;
  _camera_height_type camera_height;

   typedef double _left_dist_to_camera_type;
  _left_dist_to_camera_type left_dist_to_camera;

   typedef double _right_dist_to_camera_type;
  _right_dist_to_camera_type right_dist_to_camera;

   typedef double _front_dist_to_camera_type;
  _front_dist_to_camera_type front_dist_to_camera;

   typedef double _front_wheel_camera_dist_type;
  _front_wheel_camera_dist_type front_wheel_camera_dist;

   typedef uint32_t _camera_id_type;
  _camera_id_type camera_id;





  typedef boost::shared_ptr< ::mviz_apa_show::CalibParam_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::CalibParam_<ContainerAllocator> const> ConstPtr;

}; // struct CalibParam_

typedef ::mviz_apa_show::CalibParam_<std::allocator<void> > CalibParam;

typedef boost::shared_ptr< ::mviz_apa_show::CalibParam > CalibParamPtr;
typedef boost::shared_ptr< ::mviz_apa_show::CalibParam const> CalibParamConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::CalibParam_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::CalibParam_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::CalibParam_<ContainerAllocator1> & lhs, const ::mviz_apa_show::CalibParam_<ContainerAllocator2> & rhs)
{
  return lhs.camera_pitch == rhs.camera_pitch &&
    lhs.camera_yaw == rhs.camera_yaw &&
    lhs.camera_roll == rhs.camera_roll &&
    lhs.camera_cu == rhs.camera_cu &&
    lhs.camera_cv == rhs.camera_cv &&
    lhs.camera_fov_h == rhs.camera_fov_h &&
    lhs.camera_fov_w == rhs.camera_fov_w &&
    lhs.camera_height == rhs.camera_height &&
    lhs.left_dist_to_camera == rhs.left_dist_to_camera &&
    lhs.right_dist_to_camera == rhs.right_dist_to_camera &&
    lhs.front_dist_to_camera == rhs.front_dist_to_camera &&
    lhs.front_wheel_camera_dist == rhs.front_wheel_camera_dist &&
    lhs.camera_id == rhs.camera_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::CalibParam_<ContainerAllocator1> & lhs, const ::mviz_apa_show::CalibParam_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::CalibParam_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::CalibParam_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::CalibParam_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c0a7568b0b2e6606c86ac62a150716e0";
  }

  static const char* value(const ::mviz_apa_show::CalibParam_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc0a7568b0b2e6606ULL;
  static const uint64_t static_value2 = 0xc86ac62a150716e0ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/CalibParam";
  }

  static const char* value(const ::mviz_apa_show::CalibParam_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 camera_pitch\n"
"float64 camera_yaw\n"
"float64 camera_roll\n"
"float64 camera_cu\n"
"float64 camera_cv\n"
"float64 camera_fov_h\n"
"float64 camera_fov_w\n"
"float64 camera_height\n"
"float64 left_dist_to_camera\n"
"float64 right_dist_to_camera\n"
"float64 front_dist_to_camera\n"
"float64 front_wheel_camera_dist\n"
"uint32  camera_id                                   # 摄像头id \n"
;
  }

  static const char* value(const ::mviz_apa_show::CalibParam_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.camera_pitch);
      stream.next(m.camera_yaw);
      stream.next(m.camera_roll);
      stream.next(m.camera_cu);
      stream.next(m.camera_cv);
      stream.next(m.camera_fov_h);
      stream.next(m.camera_fov_w);
      stream.next(m.camera_height);
      stream.next(m.left_dist_to_camera);
      stream.next(m.right_dist_to_camera);
      stream.next(m.front_dist_to_camera);
      stream.next(m.front_wheel_camera_dist);
      stream.next(m.camera_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CalibParam_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::CalibParam_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::CalibParam_<ContainerAllocator>& v)
  {
    s << indent << "camera_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.camera_pitch);
    s << indent << "camera_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.camera_yaw);
    s << indent << "camera_roll: ";
    Printer<double>::stream(s, indent + "  ", v.camera_roll);
    s << indent << "camera_cu: ";
    Printer<double>::stream(s, indent + "  ", v.camera_cu);
    s << indent << "camera_cv: ";
    Printer<double>::stream(s, indent + "  ", v.camera_cv);
    s << indent << "camera_fov_h: ";
    Printer<double>::stream(s, indent + "  ", v.camera_fov_h);
    s << indent << "camera_fov_w: ";
    Printer<double>::stream(s, indent + "  ", v.camera_fov_w);
    s << indent << "camera_height: ";
    Printer<double>::stream(s, indent + "  ", v.camera_height);
    s << indent << "left_dist_to_camera: ";
    Printer<double>::stream(s, indent + "  ", v.left_dist_to_camera);
    s << indent << "right_dist_to_camera: ";
    Printer<double>::stream(s, indent + "  ", v.right_dist_to_camera);
    s << indent << "front_dist_to_camera: ";
    Printer<double>::stream(s, indent + "  ", v.front_dist_to_camera);
    s << indent << "front_wheel_camera_dist: ";
    Printer<double>::stream(s, indent + "  ", v.front_wheel_camera_dist);
    s << indent << "camera_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.camera_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_CALIBPARAM_H