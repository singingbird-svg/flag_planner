// Generated by gencpp from file bspline_race/BsplineTraj.msg
// DO NOT EDIT!


#ifndef BSPLINE_RACE_MESSAGE_BSPLINETRAJ_H
#define BSPLINE_RACE_MESSAGE_BSPLINETRAJ_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace bspline_race
{
template <class ContainerAllocator>
struct BsplineTraj_
{
  typedef BsplineTraj_<ContainerAllocator> Type;

  BsplineTraj_()
    : header()
    , position()
    , velocity()
    , acceleration()
    , yaw(0.0)
    , yaw_rate(0.0)
    , current_seq(0)  {
    }
  BsplineTraj_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)
    , yaw(0.0)
    , yaw_rate(0.0)
    , current_seq(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _position_type;
  _position_type position;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _velocity_type;
  _velocity_type velocity;

   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _acceleration_type;
  _acceleration_type acceleration;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _yaw_rate_type;
  _yaw_rate_type yaw_rate;

   typedef uint32_t _current_seq_type;
  _current_seq_type current_seq;





  typedef boost::shared_ptr< ::bspline_race::BsplineTraj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bspline_race::BsplineTraj_<ContainerAllocator> const> ConstPtr;

}; // struct BsplineTraj_

typedef ::bspline_race::BsplineTraj_<std::allocator<void> > BsplineTraj;

typedef boost::shared_ptr< ::bspline_race::BsplineTraj > BsplineTrajPtr;
typedef boost::shared_ptr< ::bspline_race::BsplineTraj const> BsplineTrajConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bspline_race::BsplineTraj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bspline_race::BsplineTraj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bspline_race::BsplineTraj_<ContainerAllocator1> & lhs, const ::bspline_race::BsplineTraj_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration &&
    lhs.yaw == rhs.yaw &&
    lhs.yaw_rate == rhs.yaw_rate &&
    lhs.current_seq == rhs.current_seq;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bspline_race::BsplineTraj_<ContainerAllocator1> & lhs, const ::bspline_race::BsplineTraj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bspline_race

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::bspline_race::BsplineTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bspline_race::BsplineTraj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bspline_race::BsplineTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bspline_race::BsplineTraj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bspline_race::BsplineTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bspline_race::BsplineTraj_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bspline_race::BsplineTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "47fe784cc6ad92a6cd7d1488e4d545d9";
  }

  static const char* value(const ::bspline_race::BsplineTraj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x47fe784cc6ad92a6ULL;
  static const uint64_t static_value2 = 0xcd7d1488e4d545d9ULL;
};

template<class ContainerAllocator>
struct DataType< ::bspline_race::BsplineTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bspline_race/BsplineTraj";
  }

  static const char* value(const ::bspline_race::BsplineTraj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bspline_race::BsplineTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"geometry_msgs/PoseStamped[] position\n"
"geometry_msgs/PoseStamped[] velocity\n"
"geometry_msgs/PoseStamped[] acceleration\n"
"\n"
"float32 yaw\n"
"float32 yaw_rate\n"
"uint32 current_seq\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::bspline_race::BsplineTraj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bspline_race::BsplineTraj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.yaw);
      stream.next(m.yaw_rate);
      stream.next(m.current_seq);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BsplineTraj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bspline_race::BsplineTraj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bspline_race::BsplineTraj_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.position[i]);
    }
    s << indent << "velocity[]" << std::endl;
    for (size_t i = 0; i < v.velocity.size(); ++i)
    {
      s << indent << "  velocity[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.velocity[i]);
    }
    s << indent << "acceleration[]" << std::endl;
    for (size_t i = 0; i < v.acceleration.size(); ++i)
    {
      s << indent << "  acceleration[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.acceleration[i]);
    }
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "yaw_rate: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_rate);
    s << indent << "current_seq: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.current_seq);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BSPLINE_RACE_MESSAGE_BSPLINETRAJ_H
