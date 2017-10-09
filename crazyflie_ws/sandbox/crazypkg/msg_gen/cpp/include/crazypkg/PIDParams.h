/* Auto-generated by genmsg_cpp for file /home/crazyfly/crazyflie_ws/sandbox/crazypkg/msg/PIDParams.msg */
#ifndef CRAZYPKG_MESSAGE_PIDPARAMS_H
#define CRAZYPKG_MESSAGE_PIDPARAMS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace crazypkg
{
template <class ContainerAllocator>
struct PIDParams_ {
  typedef PIDParams_<ContainerAllocator> Type;

  PIDParams_()
  : Kp(0.0)
  , Ki(0.0)
  , Kd(0.0)
  , N(0.0)
  {
  }

  PIDParams_(const ContainerAllocator& _alloc)
  : Kp(0.0)
  , Ki(0.0)
  , Kd(0.0)
  , N(0.0)
  {
  }

  typedef double _Kp_type;
  double Kp;

  typedef double _Ki_type;
  double Ki;

  typedef double _Kd_type;
  double Kd;

  typedef double _N_type;
  double N;


  typedef boost::shared_ptr< ::crazypkg::PIDParams_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazypkg::PIDParams_<ContainerAllocator>  const> ConstPtr;
}; // struct PIDParams
typedef  ::crazypkg::PIDParams_<std::allocator<void> > PIDParams;

typedef boost::shared_ptr< ::crazypkg::PIDParams> PIDParamsPtr;
typedef boost::shared_ptr< ::crazypkg::PIDParams const> PIDParamsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::crazypkg::PIDParams_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::crazypkg::PIDParams_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace crazypkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::crazypkg::PIDParams_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::crazypkg::PIDParams_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::crazypkg::PIDParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b4c6d11a6593f67e8239b791d98879e9";
  }

  static const char* value(const  ::crazypkg::PIDParams_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb4c6d11a6593f67eULL;
  static const uint64_t static_value2 = 0x8239b791d98879e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazypkg::PIDParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "crazypkg/PIDParams";
  }

  static const char* value(const  ::crazypkg::PIDParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::crazypkg::PIDParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 Kp\n\
float64 Ki\n\
float64 Kd\n\
float64 N\n\
\n\
";
  }

  static const char* value(const  ::crazypkg::PIDParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::crazypkg::PIDParams_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::crazypkg::PIDParams_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.Kp);
    stream.next(m.Ki);
    stream.next(m.Kd);
    stream.next(m.N);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct PIDParams_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazypkg::PIDParams_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::crazypkg::PIDParams_<ContainerAllocator> & v) 
  {
    s << indent << "Kp: ";
    Printer<double>::stream(s, indent + "  ", v.Kp);
    s << indent << "Ki: ";
    Printer<double>::stream(s, indent + "  ", v.Ki);
    s << indent << "Kd: ";
    Printer<double>::stream(s, indent + "  ", v.Kd);
    s << indent << "N: ";
    Printer<double>::stream(s, indent + "  ", v.N);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CRAZYPKG_MESSAGE_PIDPARAMS_H

