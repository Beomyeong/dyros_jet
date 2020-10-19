// Generated by gencpp from file mujoco_ros_msgs/SimStatus.msg
// DO NOT EDIT!


#ifndef MUJOCO_ROS_MSGS_MESSAGE_SIMSTATUS_H
#define MUJOCO_ROS_MSGS_MESSAGE_SIMSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <mujoco_ros_msgs/SensorBase.h>

namespace mujoco_ros_msgs
{
template <class ContainerAllocator>
struct SimStatus_
{
  typedef SimStatus_<ContainerAllocator> Type;

  SimStatus_()
    : header()
    , time(0.0)
    , name()
    , position()
    , velocity()
    , effort()
    , sensor()  {
    }
  SimStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , time(0.0)
    , name(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , effort(_alloc)
    , sensor(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _time_type;
  _time_type time;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _position_type;
  _position_type position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _velocity_type;
  _velocity_type velocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _effort_type;
  _effort_type effort;

   typedef std::vector< ::mujoco_ros_msgs::SensorBase_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::mujoco_ros_msgs::SensorBase_<ContainerAllocator> >::other >  _sensor_type;
  _sensor_type sensor;





  typedef boost::shared_ptr< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> const> ConstPtr;

}; // struct SimStatus_

typedef ::mujoco_ros_msgs::SimStatus_<std::allocator<void> > SimStatus;

typedef boost::shared_ptr< ::mujoco_ros_msgs::SimStatus > SimStatusPtr;
typedef boost::shared_ptr< ::mujoco_ros_msgs::SimStatus const> SimStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mujoco_ros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'mujoco_ros_msgs': ['/home/beom/catkin_ws/src/mujoco_ros_sim/mujoco_ros_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "78d7999bef5de3a2f4c674bc813d9460";
  }

  static const char* value(const ::mujoco_ros_msgs::SimStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x78d7999bef5de3a2ULL;
  static const uint64_t static_value2 = 0xf4c674bc813d9460ULL;
};

template<class ContainerAllocator>
struct DataType< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mujoco_ros_msgs/SimStatus";
  }

  static const char* value(const ::mujoco_ros_msgs::SimStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
Header header\n\
float64 time\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
\n\
SensorBase[] sensor\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: mujoco_ros_msgs/SensorBase\n\
string name\n\
\n\
float64[] data\n\
";
  }

  static const char* value(const ::mujoco_ros_msgs::SimStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.time);
      stream.next(m.name);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.effort);
      stream.next(m.sensor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mujoco_ros_msgs::SimStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mujoco_ros_msgs::SimStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
    s << indent << "sensor[]" << std::endl;
    for (size_t i = 0; i < v.sensor.size(); ++i)
    {
      s << indent << "  sensor[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mujoco_ros_msgs::SensorBase_<ContainerAllocator> >::stream(s, indent + "    ", v.sensor[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MUJOCO_ROS_MSGS_MESSAGE_SIMSTATUS_H
