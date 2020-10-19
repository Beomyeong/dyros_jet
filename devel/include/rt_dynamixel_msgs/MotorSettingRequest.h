// Generated by gencpp from file rt_dynamixel_msgs/MotorSettingRequest.msg
// DO NOT EDIT!


#ifndef RT_DYNAMIXEL_MSGS_MESSAGE_MOTORSETTINGREQUEST_H
#define RT_DYNAMIXEL_MSGS_MESSAGE_MOTORSETTINGREQUEST_H


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
struct MotorSettingRequest_
{
  typedef MotorSettingRequest_<ContainerAllocator> Type;

  MotorSettingRequest_()
    : mode(0)
    , id(0)
    , value(0)
    , fvalue(0.0)  {
    }
  MotorSettingRequest_(const ContainerAllocator& _alloc)
    : mode(0)
    , id(0)
    , value(0)
    , fvalue(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _mode_type;
  _mode_type mode;

   typedef int32_t _id_type;
  _id_type id;

   typedef int64_t _value_type;
  _value_type value;

   typedef double _fvalue_type;
  _fvalue_type fvalue;



  enum {
    SET_HOMING_OFFSET = 17u,
    GET_HOMING_OFFSET = 18u,
    SET_TORQUE_ENABLE = 19u,
    SET_GOAL_POSITION = 20u,
  };


  typedef boost::shared_ptr< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MotorSettingRequest_

typedef ::rt_dynamixel_msgs::MotorSettingRequest_<std::allocator<void> > MotorSettingRequest;

typedef boost::shared_ptr< ::rt_dynamixel_msgs::MotorSettingRequest > MotorSettingRequestPtr;
typedef boost::shared_ptr< ::rt_dynamixel_msgs::MotorSettingRequest const> MotorSettingRequestConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rt_dynamixel_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'rt_dynamixel_msgs': ['/home/beom/catkin_ws/src/rt_dynamixel_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa57006293ff4f56089dba8a7218e23b";
  }

  static const char* value(const ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa57006293ff4f56ULL;
  static const uint64_t static_value2 = 0x089dba8a7218e23bULL;
};

template<class ContainerAllocator>
struct DataType< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rt_dynamixel_msgs/MotorSettingRequest";
  }

  static const char* value(const ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
uint8 SET_HOMING_OFFSET=17\n\
uint8 GET_HOMING_OFFSET=18\n\
uint8 SET_TORQUE_ENABLE=19\n\
uint8 SET_GOAL_POSITION=20\n\
\n\
int32 mode\n\
int32 id\n\
int64 value\n\
float64 fvalue\n\
";
  }

  static const char* value(const ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.id);
      stream.next(m.value);
      stream.next(m.fvalue);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorSettingRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rt_dynamixel_msgs::MotorSettingRequest_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "value: ";
    Printer<int64_t>::stream(s, indent + "  ", v.value);
    s << indent << "fvalue: ";
    Printer<double>::stream(s, indent + "  ", v.fvalue);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RT_DYNAMIXEL_MSGS_MESSAGE_MOTORSETTINGREQUEST_H
