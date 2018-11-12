// Generated by gencpp from file iiwa_msgs/JointImpedanceControlMode.msg
// DO NOT EDIT!


#ifndef IIWA_MSGS_MESSAGE_JOINTIMPEDANCECONTROLMODE_H
#define IIWA_MSGS_MESSAGE_JOINTIMPEDANCECONTROLMODE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/JointQuantity.h>

namespace iiwa_msgs
{
template <class ContainerAllocator>
struct JointImpedanceControlMode_
{
  typedef JointImpedanceControlMode_<ContainerAllocator> Type;

  JointImpedanceControlMode_()
    : joint_stiffness()
    , joint_damping()  {
    }
  JointImpedanceControlMode_(const ContainerAllocator& _alloc)
    : joint_stiffness(_alloc)
    , joint_damping(_alloc)  {
  (void)_alloc;
    }



   typedef  ::iiwa_msgs::JointQuantity_<ContainerAllocator>  _joint_stiffness_type;
  _joint_stiffness_type joint_stiffness;

   typedef  ::iiwa_msgs::JointQuantity_<ContainerAllocator>  _joint_damping_type;
  _joint_damping_type joint_damping;





  typedef boost::shared_ptr< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> const> ConstPtr;

}; // struct JointImpedanceControlMode_

typedef ::iiwa_msgs::JointImpedanceControlMode_<std::allocator<void> > JointImpedanceControlMode;

typedef boost::shared_ptr< ::iiwa_msgs::JointImpedanceControlMode > JointImpedanceControlModePtr;
typedef boost::shared_ptr< ::iiwa_msgs::JointImpedanceControlMode const> JointImpedanceControlModeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace iiwa_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'iiwa_msgs': ['/home/birl/worksp/iiwa_stack_birl/src/iiwa_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c2267019473eba8c0a4619b649192c0d";
  }

  static const char* value(const ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc2267019473eba8cULL;
  static const uint64_t static_value2 = 0x0a4619b649192c0dULL;
};

template<class ContainerAllocator>
struct DataType< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "iiwa_msgs/JointImpedanceControlMode";
  }

  static const char* value(const ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Stiffness values in [Nm/rad]. Stiffness values must be >= 0. \n\
JointQuantity joint_stiffness\n\
\n\
# Damping values. Damping values must be between 0 and 1. \n\
JointQuantity joint_damping\n\
\n\
================================================================================\n\
MSG: iiwa_msgs/JointQuantity\n\
float32 a1\n\
float32 a2\n\
float32 a3\n\
float32 a4\n\
float32 a5\n\
float32 a6\n\
float32 a7\n\
";
  }

  static const char* value(const ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_stiffness);
      stream.next(m.joint_damping);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JointImpedanceControlMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::iiwa_msgs::JointImpedanceControlMode_<ContainerAllocator>& v)
  {
    s << indent << "joint_stiffness: ";
    s << std::endl;
    Printer< ::iiwa_msgs::JointQuantity_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_stiffness);
    s << indent << "joint_damping: ";
    s << std::endl;
    Printer< ::iiwa_msgs::JointQuantity_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_damping);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IIWA_MSGS_MESSAGE_JOINTIMPEDANCECONTROLMODE_H
