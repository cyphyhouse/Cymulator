#ifndef ROS_IGN_BRIDGE__CONVERT_MARKER_INTERFACES_HPP_
#define ROS_IGN_BRIDGE__CONVERT_MARKER_INTERFACES_HPP_

// include Ignition builtin messages
#include <ignition/msgs.hh>

#include "cym_marker/Marker.h"

#include "ros_ign_bridge/convert_decl.hpp"

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(
  const cym_marker::Marker & ros_msg,
  ignition::msgs::Marker & ign_msg);

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Marker & ign_msg,
  cym_marker::Marker & ros_msg);

}

#endif  // ROS_IGN_BRIDGE__CONVERT_MARKER_INTERFACES_HPP_
