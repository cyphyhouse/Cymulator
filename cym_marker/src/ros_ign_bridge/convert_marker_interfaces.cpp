#include <ros/console.h>

#include "ros_ign_bridge/convert_marker_interfaces.hpp"

namespace ros_ign_bridge
{
template<>
void
convert_ros_to_ign(
  const cym_marker::Script & ros_msg,
  ignition::msgs::Material::Script & ign_msg)
{
  // TODO test list
  for(auto uri : ros_msg.uri)
  {
    *ign_msg.add_uri() = uri;
  }
  ign_msg.set_name(ros_msg.name);
}

template<>
void
convert_ros_to_ign(
  const cym_marker::Material & ros_msg,
  ignition::msgs::Material & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, *ign_msg.mutable_header());
  convert_ros_to_ign(ros_msg.script, *ign_msg.mutable_script());
  if (ros_msg.shader_type == 0)
  {
    // SKIP
  }
  else if (ros_msg.shader_type == cym_marker::Material::VERTEX)
  {
    ign_msg.set_shader_type(ignition::msgs::Material::VERTEX);
  }
  else if (ros_msg.shader_type == cym_marker::Material::PIXEL)
  {
    ign_msg.set_shader_type(ignition::msgs::Material::PIXEL);
  }
  else if (ros_msg.shader_type == cym_marker::Material::NORMAL_MAP_OBJECT_SPACE)
  {
    ign_msg.set_shader_type(ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE);
  }
  else if (ros_msg.shader_type == cym_marker::Material::NORMAL_MAP_TANGENT_SPACE)
  {
    ign_msg.set_shader_type(ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported material shader type [" << ros_msg.shader_type << "]"
              << std::endl);
  }
  ign_msg.set_normal_map(ros_msg.normal_map);
  convert_ros_to_ign(ros_msg.ambient, *ign_msg.mutable_ambient());
  convert_ros_to_ign(ros_msg.diffuse, *ign_msg.mutable_diffuse());
  convert_ros_to_ign(ros_msg.specular, *ign_msg.mutable_specular());
  convert_ros_to_ign(ros_msg.emissive, *ign_msg.mutable_emissive());
  ign_msg.set_lighting(ros_msg.lighting);
}

template<>
void
convert_ros_to_ign(
  const cym_marker::Marker & ros_msg,
  ignition::msgs::Marker & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  if(ros_msg.action == cym_marker::Marker::ADD_MODIFY)
  {
    ign_msg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  }
  else if (ros_msg.action == cym_marker::Marker::DELETE_MARKER)
  {
    ign_msg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  }
  else if (ros_msg.action == cym_marker::Marker::DELETE_ALL)
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
  ign_msg.set_layer(ros_msg.layer);

  if(ros_msg.type == cym_marker::Marker::NONE)
  {
    ign_msg.set_type(ignition::msgs::Marker::NONE);
  }
  else if (ros_msg.type == cym_marker::Marker::BOX)
  {
    ign_msg.set_type(ignition::msgs::Marker::BOX);
  }
  else if (ros_msg.type == cym_marker::Marker::CYLINDER)
  {
    ign_msg.set_type(ignition::msgs::Marker::CYLINDER);
  }
  else if (ros_msg.type == cym_marker::Marker::LINE_LIST)
  {
    ign_msg.set_type(ignition::msgs::Marker::LINE_LIST);
  }
  else if (ros_msg.type == cym_marker::Marker::LINE_STRIP)
  {
    ign_msg.set_type(ignition::msgs::Marker::LINE_STRIP);
  }
  else if (ros_msg.type == cym_marker::Marker::POINTS)
  {
    ign_msg.set_type(ignition::msgs::Marker::POINTS);
  }
  else if (ros_msg.type == cym_marker::Marker::SPHERE)
  {
    ign_msg.set_type(ignition::msgs::Marker::SPHERE);
  }
  else if (ros_msg.type == cym_marker::Marker::TEXT)
  {
    ign_msg.set_type(ignition::msgs::Marker::TEXT);
  }
  else if (ros_msg.type == cym_marker::Marker::TRIANGLE_FAN)
  {
    ign_msg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  }
  else if (ros_msg.type == cym_marker::Marker::TRIANGLE_LIST)
  {
    ign_msg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  }
  else if (ros_msg.type == cym_marker::Marker::TRIANGLE_STRIP)
  {
    ign_msg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
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
  convert_ros_to_ign(ros_msg.material, *ign_msg.mutable_material());


  // TODO Test if array works
  for(auto point : ros_msg.point)
  {
    convert_ros_to_ign(point, *ign_msg.add_point());
  }

  ign_msg.set_text(ros_msg.text);
  ign_msg.set_parent(ros_msg.parent);
  if (ros_msg.visibility == cym_marker::Marker::GUI)
  {
    ign_msg.set_visibility(ignition::msgs::Marker::GUI);
  }
  else if (ros_msg.visibility == cym_marker::Marker::ALL)
  {
    ign_msg.set_visibility(ignition::msgs::Marker::ALL);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported marker visibility [" << ros_msg.visibility << "]"
              << std::endl);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Marker & ign_msg,
  cym_marker::Marker & ros_msg)
{
    //TODO
}

}

