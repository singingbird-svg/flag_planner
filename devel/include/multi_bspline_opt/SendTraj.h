// Generated by gencpp from file multi_bspline_opt/SendTraj.msg
// DO NOT EDIT!


#ifndef MULTI_BSPLINE_OPT_MESSAGE_SENDTRAJ_H
#define MULTI_BSPLINE_OPT_MESSAGE_SENDTRAJ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace multi_bspline_opt
{
template <class ContainerAllocator>
struct SendTraj_
{
  typedef SendTraj_<ContainerAllocator> Type;

  SendTraj_()
    : drone_id(0)
    , traj_id(0)
    , order(0)
    , cps_num_(0)
    , start_time()
    , start_pos_x(0.0)
    , start_pos_y(0.0)
    , start_vel_x(0.0)
    , start_vel_y(0.0)
    , end_pos_x(0.0)
    , end_pos_y(0.0)
    , control_pts()
    , knots()  {
    }
  SendTraj_(const ContainerAllocator& _alloc)
    : drone_id(0)
    , traj_id(0)
    , order(0)
    , cps_num_(0)
    , start_time()
    , start_pos_x(0.0)
    , start_pos_y(0.0)
    , start_vel_x(0.0)
    , start_vel_y(0.0)
    , end_pos_x(0.0)
    , end_pos_y(0.0)
    , control_pts(_alloc)
    , knots(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _drone_id_type;
  _drone_id_type drone_id;

   typedef int64_t _traj_id_type;
  _traj_id_type traj_id;

   typedef int32_t _order_type;
  _order_type order;

   typedef int32_t _cps_num__type;
  _cps_num__type cps_num_;

   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef double _start_pos_x_type;
  _start_pos_x_type start_pos_x;

   typedef double _start_pos_y_type;
  _start_pos_y_type start_pos_y;

   typedef double _start_vel_x_type;
  _start_vel_x_type start_vel_x;

   typedef double _start_vel_y_type;
  _start_vel_y_type start_vel_y;

   typedef double _end_pos_x_type;
  _end_pos_x_type end_pos_x;

   typedef double _end_pos_y_type;
  _end_pos_y_type end_pos_y;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> >> _control_pts_type;
  _control_pts_type control_pts;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _knots_type;
  _knots_type knots;





  typedef boost::shared_ptr< ::multi_bspline_opt::SendTraj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multi_bspline_opt::SendTraj_<ContainerAllocator> const> ConstPtr;

}; // struct SendTraj_

typedef ::multi_bspline_opt::SendTraj_<std::allocator<void> > SendTraj;

typedef boost::shared_ptr< ::multi_bspline_opt::SendTraj > SendTrajPtr;
typedef boost::shared_ptr< ::multi_bspline_opt::SendTraj const> SendTrajConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multi_bspline_opt::SendTraj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::multi_bspline_opt::SendTraj_<ContainerAllocator1> & lhs, const ::multi_bspline_opt::SendTraj_<ContainerAllocator2> & rhs)
{
  return lhs.drone_id == rhs.drone_id &&
    lhs.traj_id == rhs.traj_id &&
    lhs.order == rhs.order &&
    lhs.cps_num_ == rhs.cps_num_ &&
    lhs.start_time == rhs.start_time &&
    lhs.start_pos_x == rhs.start_pos_x &&
    lhs.start_pos_y == rhs.start_pos_y &&
    lhs.start_vel_x == rhs.start_vel_x &&
    lhs.start_vel_y == rhs.start_vel_y &&
    lhs.end_pos_x == rhs.end_pos_x &&
    lhs.end_pos_y == rhs.end_pos_y &&
    lhs.control_pts == rhs.control_pts &&
    lhs.knots == rhs.knots;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::multi_bspline_opt::SendTraj_<ContainerAllocator1> & lhs, const ::multi_bspline_opt::SendTraj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace multi_bspline_opt

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multi_bspline_opt::SendTraj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multi_bspline_opt::SendTraj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multi_bspline_opt::SendTraj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cdb4d98dc8568f5ace0ee75482493f76";
  }

  static const char* value(const ::multi_bspline_opt::SendTraj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcdb4d98dc8568f5aULL;
  static const uint64_t static_value2 = 0xce0ee75482493f76ULL;
};

template<class ContainerAllocator>
struct DataType< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multi_bspline_opt/SendTraj";
  }

  static const char* value(const ::multi_bspline_opt::SendTraj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 drone_id\n"
"int64 traj_id\n"
"int32 order\n"
"int32 cps_num_\n"
"# int32 Dim_\n"
"# int32 TrajSampleRate\n"
"# float64 beta\n"
"time start_time\n"
"\n"
"\n"
"float64 start_pos_x\n"
"float64 start_pos_y\n"
"float64 start_vel_x\n"
"float64 start_vel_y\n"
"float64 end_pos_x\n"
"float64 end_pos_y\n"
"# float64 yaw_rate\n"
"\n"
"geometry_msgs/Point[] control_pts\n"
"float64[] knots\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::multi_bspline_opt::SendTraj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.drone_id);
      stream.next(m.traj_id);
      stream.next(m.order);
      stream.next(m.cps_num_);
      stream.next(m.start_time);
      stream.next(m.start_pos_x);
      stream.next(m.start_pos_y);
      stream.next(m.start_vel_x);
      stream.next(m.start_vel_y);
      stream.next(m.end_pos_x);
      stream.next(m.end_pos_y);
      stream.next(m.control_pts);
      stream.next(m.knots);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SendTraj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multi_bspline_opt::SendTraj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multi_bspline_opt::SendTraj_<ContainerAllocator>& v)
  {
    s << indent << "drone_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.drone_id);
    s << indent << "traj_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.traj_id);
    s << indent << "order: ";
    Printer<int32_t>::stream(s, indent + "  ", v.order);
    s << indent << "cps_num_: ";
    Printer<int32_t>::stream(s, indent + "  ", v.cps_num_);
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "start_pos_x: ";
    Printer<double>::stream(s, indent + "  ", v.start_pos_x);
    s << indent << "start_pos_y: ";
    Printer<double>::stream(s, indent + "  ", v.start_pos_y);
    s << indent << "start_vel_x: ";
    Printer<double>::stream(s, indent + "  ", v.start_vel_x);
    s << indent << "start_vel_y: ";
    Printer<double>::stream(s, indent + "  ", v.start_vel_y);
    s << indent << "end_pos_x: ";
    Printer<double>::stream(s, indent + "  ", v.end_pos_x);
    s << indent << "end_pos_y: ";
    Printer<double>::stream(s, indent + "  ", v.end_pos_y);
    s << indent << "control_pts[]" << std::endl;
    for (size_t i = 0; i < v.control_pts.size(); ++i)
    {
      s << indent << "  control_pts[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.control_pts[i]);
    }
    s << indent << "knots[]" << std::endl;
    for (size_t i = 0; i < v.knots.size(); ++i)
    {
      s << indent << "  knots[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.knots[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTI_BSPLINE_OPT_MESSAGE_SENDTRAJ_H