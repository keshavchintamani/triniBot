// Generated by gencpp from file balltracker/command.msg
// DO NOT EDIT!


#ifndef BALLTRACKER_MESSAGE_COMMAND_H
#define BALLTRACKER_MESSAGE_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace balltracker
{
template <class ContainerAllocator>
struct command_
{
  typedef command_<ContainerAllocator> Type;

  command_()
    : priority()
    , direction()
    , speed(0)
    , angle(0.0)  {
    }
  command_(const ContainerAllocator& _alloc)
    : priority(_alloc)
    , direction(_alloc)
    , speed(0)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _priority_type;
  _priority_type priority;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _direction_type;
  _direction_type direction;

   typedef int32_t _speed_type;
  _speed_type speed;

   typedef float _angle_type;
  _angle_type angle;




  typedef boost::shared_ptr< ::balltracker::command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::balltracker::command_<ContainerAllocator> const> ConstPtr;

}; // struct command_

typedef ::balltracker::command_<std::allocator<void> > command;

typedef boost::shared_ptr< ::balltracker::command > commandPtr;
typedef boost::shared_ptr< ::balltracker::command const> commandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::balltracker::command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::balltracker::command_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace balltracker

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'balltracker': ['/home/kc/sandbox/triniBot/tbros/src/balltracker/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::balltracker::command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::balltracker::command_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::balltracker::command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::balltracker::command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balltracker::command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::balltracker::command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::balltracker::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "42cf6ba923645af9d43237e750433163";
  }

  static const char* value(const ::balltracker::command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x42cf6ba923645af9ULL;
  static const uint64_t static_value2 = 0xd43237e750433163ULL;
};

template<class ContainerAllocator>
struct DataType< ::balltracker::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "balltracker/command";
  }

  static const char* value(const ::balltracker::command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::balltracker::command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string priority\n\
string direction\n\
int32 speed\n\
float32 angle\n\
";
  }

  static const char* value(const ::balltracker::command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::balltracker::command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.priority);
      stream.next(m.direction);
      stream.next(m.speed);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::balltracker::command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::balltracker::command_<ContainerAllocator>& v)
  {
    s << indent << "priority: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.priority);
    s << indent << "direction: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.direction);
    s << indent << "speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.speed);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BALLTRACKER_MESSAGE_COMMAND_H
