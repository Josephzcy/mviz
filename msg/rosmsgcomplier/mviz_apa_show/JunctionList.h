// Generated by gencpp from file mviz_apa_show/JunctionList.msg
// DO NOT EDIT!


#ifndef MVIZ_APA_SHOW_MESSAGE_JUNCTIONLIST_H
#define MVIZ_APA_SHOW_MESSAGE_JUNCTIONLIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mviz_apa_show/Junction.h>

namespace mviz_apa_show
{
template <class ContainerAllocator>
struct JunctionList_
{
  typedef JunctionList_<ContainerAllocator> Type;

  JunctionList_()
    : junc_list()  {
    }
  JunctionList_(const ContainerAllocator& _alloc)
    : junc_list(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::mviz_apa_show::Junction_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mviz_apa_show::Junction_<ContainerAllocator> >::other >  _junc_list_type;
  _junc_list_type junc_list;





  typedef boost::shared_ptr< ::mviz_apa_show::JunctionList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mviz_apa_show::JunctionList_<ContainerAllocator> const> ConstPtr;

}; // struct JunctionList_

typedef ::mviz_apa_show::JunctionList_<std::allocator<void> > JunctionList;

typedef boost::shared_ptr< ::mviz_apa_show::JunctionList > JunctionListPtr;
typedef boost::shared_ptr< ::mviz_apa_show::JunctionList const> JunctionListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mviz_apa_show::JunctionList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mviz_apa_show::JunctionList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mviz_apa_show::JunctionList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::JunctionList_<ContainerAllocator2> & rhs)
{
  return lhs.junc_list == rhs.junc_list;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mviz_apa_show::JunctionList_<ContainerAllocator1> & lhs, const ::mviz_apa_show::JunctionList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mviz_apa_show

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mviz_apa_show::JunctionList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mviz_apa_show::JunctionList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mviz_apa_show::JunctionList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d5ee3457cf3098e468ad13bc191d0e42";
  }

  static const char* value(const ::mviz_apa_show::JunctionList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd5ee3457cf3098e4ULL;
  static const uint64_t static_value2 = 0x68ad13bc191d0e42ULL;
};

template<class ContainerAllocator>
struct DataType< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mviz_apa_show/JunctionList";
  }

  static const char* value(const ::mviz_apa_show::JunctionList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Junction[] junc_list\n"
"================================================================================\n"
"MSG: mviz_apa_show/Junction\n"
"# 道路合流分流点, 总字节数64\n"
"# Type type = 1                    # 类型 字节数:4\n"
"Point2f pt_image_coord     # 图像坐标 字节数:8 \n"
"Point2f pt_vehicle_coord   # 车身坐标 字节数:8 \n"
"uint64[] laneline_id                   # 形成交叉点的车道线id  字节数:32 (最多4个)\n"
"int32 state                                   # 状态 字节数:4 \n"
"float32 confidence                              # 置信度 字节数:4\n"
"int32 pos                                       # 位置  字节数：4 1-> 左边 2-> 右边\n"
"================================================================================\n"
"MSG: mviz_apa_show/Point2f\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::mviz_apa_show::JunctionList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.junc_list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JunctionList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mviz_apa_show::JunctionList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mviz_apa_show::JunctionList_<ContainerAllocator>& v)
  {
    s << indent << "junc_list[]" << std::endl;
    for (size_t i = 0; i < v.junc_list.size(); ++i)
    {
      s << indent << "  junc_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mviz_apa_show::Junction_<ContainerAllocator> >::stream(s, indent + "    ", v.junc_list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MVIZ_APA_SHOW_MESSAGE_JUNCTIONLIST_H
