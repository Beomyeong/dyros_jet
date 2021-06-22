// Generated by gencpp from file rt_dynamixel_msgs/JointSet.msg
// DO NOT EDIT!


#ifndef RT_DYNAMIXEL_MSGS_MESSAGE_JOINTSET_H
#define RT_DYNAMIXEL_MSGS_MESSAGE_JOINTSET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rt_dynamixel_msgs
{
template <class ContainerAllocator>
struct JointSet_
{
  typedef JointSet_<ContainerAllocator> Type;

  JointSet_()
    : id()
    , angle()  {
    }
  JointSet_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , angle(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _id_type;
  _id_type id;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> const> ConstPtr;

}; // struct JointSet_

typedef ::rt_dynamixel_msgs::JointSet_<std::allocator<void> > JointSet;

typedef boost::shared_ptr< ::rt_dynamixel_msgs::JointSet > JointSetPtr;
typedef boost::shared_ptr< ::rt_dynamixel_msgs::JointSet const> JointSetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rt_dynamixel_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rt_dynamixel_msgs': ['/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b64b5c2f49125ccc72173634fc438fd8";
  }

  static const char* value(const ::rt_dynamixel_msgs::JointSet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb64b5c2f49125cccULL;
  static const uint64_t static_value2 = 0x72173634fc438fd8ULL;
};

template<class ContainerAllocator>
struct DataType< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rt_dynamixel_msgs/JointSet";
  }

  static const char* value(const ::rt_dynamixel_msgs::JointSet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8[] id\n\
float64[] angle\n\
\n\
";
  }

  static const char* value(const ::rt_dynamixel_msgs::JointSet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointSet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rt_dynamixel_msgs::JointSet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rt_dynamixel_msgs::JointSet_<ContainerAllocator>& v)
  {
    s << indent << "id[]" << std::endl;
    for (size_t i = 0; i < v.id.size(); ++i)
    {
      s << indent << "  id[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.id[i]);
    }
    s << indent << "angle[]" << std::endl;
    for (size_t i = 0; i < v.angle.size(); ++i)
    {
      s << indent << "  angle[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.angle[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RT_DYNAMIXEL_MSGS_MESSAGE_JOINTSET_H
