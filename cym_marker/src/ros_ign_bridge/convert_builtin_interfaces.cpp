// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_ign_bridge/convert_builtin_interfaces.hpp"

namespace ros_ign_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(const std::string &input,
                              const std::string &old_delim,
                              const std::string new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size())
  {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos)
    {
      output += new_delim;
      pos += old_delim.size();
    }

    last_pos = pos;
  }

  return output;
}

// Frame id from ROS to ign is not supported right now
// std::string frame_id_ros_to_ign(const std::string &frame_id)
// {
//   return replace_delimiter(frame_id, "/", "::");
// }

std::string frame_id_ign_to_ros(const std::string &frame_id)
{
  return replace_delimiter(frame_id, "::", "/");
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::ColorRGBA & ros_msg,
  ignition::msgs::Color & ign_msg)
{
  ign_msg.set_r(ros_msg.r);
  ign_msg.set_g(ros_msg.g);
  ign_msg.set_b(ros_msg.b);
  ign_msg.set_a(ros_msg.a);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Color & ign_msg,
  std_msgs::ColorRGBA & ros_msg)
{
  ros_msg.r = ign_msg.r();
  ros_msg.g = ign_msg.g();
  ros_msg.b = ign_msg.b();
  ros_msg.a = ign_msg.a();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Empty &,
  ignition::msgs::Empty &)
{
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Empty &,
  std_msgs::Empty &)
{
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros_msg.stamp.sec);
  ign_msg.mutable_stamp()->set_nsec(ros_msg.stamp.nsec);
  auto newPair = ign_msg.add_data();
  newPair->set_key("seq");
  newPair->add_value(std::to_string(ros_msg.seq));
  newPair = ign_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros_msg.frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg)
{
  ros_msg.stamp = ros::Time(ign_msg.stamp().sec(), ign_msg.stamp().nsec());
  for (auto i = 0; i < ign_msg.data_size(); ++i)
  {
    auto aPair = ign_msg.data(i);
    if (aPair.key() == "seq" && aPair.value_size() > 0)
    {
      std::string value = aPair.value(0);
      try
      {
        unsigned long ul = std::stoul(value, nullptr);
        ros_msg.seq = ul;
      }
      catch (std::exception & e)
      {
        ROS_ERROR_STREAM("Exception converting [" << value << "] to an "
                  << "unsigned int" << std::endl);
      }
    }
    else if (aPair.key() == "frame_id" && aPair.value_size() > 0)
    {
      ros_msg.frame_id = frame_id_ign_to_ros(aPair.value(0));
    }
  }
}

template<>
void
convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros_msg)
{
  ros_msg.clock = ros::Time(ign_msg.sim().sec(), ign_msg.sim().nsec());
}

template<>
void
convert_ros_to_ign(
  const rosgraph_msgs::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg)
{
  ign_msg.mutable_sim()->set_sec(ros_msg.clock.sec);
  ign_msg.mutable_sim()->set_nsec(ros_msg.clock.nsec);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
  ign_msg.set_w(ros_msg.w);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
  ros_msg.w = ign_msg.w();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.position, *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.orientation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.position);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose, ign_msg);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.translation , *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.rotation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.translation);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.transform, ign_msg);

  auto newPair = ign_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.transform);
  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_ign(
  const visualization_msgs::Marker & ros_msg,
  ignition::msgs::Marker & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  if(ros_msg.action == visualization_msgs::Marker::ADD)
  {
    ign_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  }
  else if (ros_msg.action == visualization_msgs::Marker::MODIFY)
  {
    ign_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  }
  else if (ros_msg.action == visualization_msgs::Marker::DELETE)
  {
    ign_msg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  }
  else if (ros_msg.action == visualization_msgs::Marker::DELETEALL)
  {
    ign_msg.set_action(ignition::msgs::Marker::DELETE_ALL);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported marker action [" << ros_msg.action << "]"
              << std::endl);
  }

  ign_msg.set_ns(ros_msg.ns);
  ign_msg.set_id(ros_msg.id);
  // ign_msg.set_layer(0);  // No layer in visualization_msgs::Marker

  if(ros_msg.type == visualization_msgs::Marker::ARROW)
  {
    // TODO
  }
  else if (ros_msg.type == visualization_msgs::Marker::CUBE)
  {
    ign_msg.set_type(ignition::msgs::Marker::BOX);
  }
  else if (ros_msg.type == visualization_msgs::Marker::SPHERE)
  {
    ign_msg.set_type(ignition::msgs::Marker::SPHERE);
  }
  else if (ros_msg.type == visualization_msgs::Marker::CYLINDER)
  {
    ign_msg.set_type(ignition::msgs::Marker::CYLINDER);
  }
  else if (ros_msg.type == visualization_msgs::Marker::LINE_STRIP)
  {
    ign_msg.set_type(ignition::msgs::Marker::LINE_STRIP);
  }
  else if (ros_msg.type == visualization_msgs::Marker::LINE_LIST)
  {
    ign_msg.set_type(ignition::msgs::Marker::LINE_LIST);
  }
  else if (ros_msg.type == visualization_msgs::Marker::CUBE_LIST)
  {
    // TODO
  }
  else if (ros_msg.type == visualization_msgs::Marker::SPHERE_LIST)
  {
    // TODO
  }
  else if (ros_msg.type == visualization_msgs::Marker::POINTS)
  {
    ign_msg.set_type(ignition::msgs::Marker::POINTS);
  }
  else if (ros_msg.type == visualization_msgs::Marker::TEXT_VIEW_FACING)
  {
    ign_msg.set_type(ignition::msgs::Marker::TEXT);
  }
  else if (ros_msg.type == visualization_msgs::Marker::MESH_RESOURCE)
  {
    // TODO
  }
  else if (ros_msg.type == visualization_msgs::Marker::TRIANGLE_LIST)
  {
    ign_msg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported marker type [" << ros_msg.type << "]"
              << std::endl);
  }


  if (!ros_msg.lifetime.isZero())
  {
    ign_msg.mutable_lifetime()->set_sec(ros_msg.lifetime.sec);
    ign_msg.mutable_lifetime()->set_nsec(ros_msg.lifetime.nsec);
    ROS_WARN_STREAM(
        "Gazebo marker lifetime is unstable due to inconsistent SimTime inside Gazebo. "
        << "Please use with caution."
        << std::endl);
  }

  convert_ros_to_ign(ros_msg.pose, *ign_msg.mutable_pose());
  convert_ros_to_ign(ros_msg.scale, *ign_msg.mutable_scale());

  //TODO ign_msg.set_material();

  // TODO Test if array works
  for(size_t i=0; i < ros_msg.points.size(); ++i)
  {
    convert_ros_to_ign(ros_msg.points[i], *ign_msg.add_point());
  }

  ign_msg.set_text(ros_msg.text);
  // ign_msg.set_parent("");  // No parent in visualization_msgs::Marker
  ign_msg.set_visibility(ignition::msgs::Marker::GUI);  // Only users see markers
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Marker & ign_msg,
  visualization_msgs::Marker & ros_msg)
{
    //TODO
}

}  // namespace ros_ign_bridge
