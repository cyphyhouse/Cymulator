/*
 * ROS node providing marker service in Gazebo by sending corresponding requests
 * to Inginition Transport nodes
 *
 * Code modified from
 * https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/marker/marker.cc
 * https://bitbucket.org/osrf/gazebo/src/default/tools/gz_marker.cc
*/

#include <bitset>
#include <iostream>
#include <string>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_aggregator/status_item.h>

#include <ros_ign_bridge/convert_marker_interfaces.hpp>

namespace {
typedef cym_marker::Marker ROSMarker;
typedef ignition::msgs::Marker IGNMarker;
typedef ignition::msgs::Vector3d IGNPoint;


void addHollowBox(IGNMarker& msg,
                  const std::array<IGNPoint, 2>& corner)
{
    typedef std::bitset<3> Vertex3d;

    // Construct 4 x 3 = 12 edges of the rectangle
    const std::array<const Vertex3d, 4> vertice = {0b000, 0b110, 0b101, 0b011};
    for(const auto src: vertice)
    {
        // Create source point
        IGNPoint p0;
        p0.set_x(corner[ src[0] ].x());
        p0.set_y(corner[ src[1] ].y());
        p0.set_z(corner[ src[2] ].z());

        const std::array<const Vertex3d, 3> targets = {
            Vertex3d(src).flip(0),
            Vertex3d(src).flip(1),
            Vertex3d(src).flip(2)};
        for(auto tgt: targets)
        {
            // Add source point by copying
            *msg.add_point() = p0;
            // Create and directly set target point
            IGNPoint& p1 = *msg.add_point(); 
            p1.set_x(corner[ tgt[0] ].x());
            p1.set_y(corner[ tgt[1] ].y());
            p1.set_z(corner[ tgt[2] ].z());
        }
    }
    return;
}

const std::map<std::string, std::string> PREDEFINED_SCRIPT = {
    {"drone0", "Gazebo/OrangeTransparent"},
    {"drone1", "Gazebo/YellowTransparent"},
    {"drone2", "Gazebo/BlueTransparent"},
    {"drone3", "Gazebo/DarkMagentaTransparent"},
    {"drone4", "Gazebo/GreyTransparent"},
    {"drone5", "Gazebo/BlackTransparent"},
};

bool setIGNMarker(IGNMarker& msg,
                  const diagnostic_aggregator::StatusItem& status_item)
{
    msg.set_ns(status_item.getHwId());
    msg.set_id(0);
    msg.set_type(IGNMarker::LINE_LIST);
    auto& mat = *msg.mutable_material();
    auto& script = *mat.mutable_script();

    if(status_item.getLevel() == diagnostic_aggregator::DiagnosticLevel::Level_OK)
    {
        if(PREDEFINED_SCRIPT.count(status_item.getHwId()))
        {   script.set_name(PREDEFINED_SCRIPT.at(status_item.getHwId()));}
        else
        {   script.set_name("Gazebo/RedTransparent");}
    }
    else
    {
        script.set_name("Gazebo/RedTransparent");
    }

    try 
    {
        YAML::Node state_list = YAML::Load(status_item.getValue("data"));
        // TODO Generalize to other kinds of reachset representations
        for(auto it = state_list.begin(); it != state_list.end(); ++it)
        {
            const YAML::Node& state_bound0 = *it;
            ++it;
            if(it == state_list.end())
            {   break;}
            const YAML::Node& state_bound1 = *it;
            // TODO think about how to use time step
            // const double& step = state_bound0[0].as<double>();
            std::array<IGNPoint, 2> p;
            p[0].set_x(state_bound0[1].as<double>());
            p[0].set_y(state_bound0[2].as<double>());
            p[0].set_z(state_bound0[3].as<double>());

            p[1].set_x(state_bound1[1].as<double>());
            p[1].set_y(state_bound1[2].as<double>());
            p[1].set_z(state_bound1[3].as<double>());

            addHollowBox(msg, p);
        }
        return true;
    }
    catch(YAML::ParserException& e)
    {
        ROS_WARN_STREAM(
            "Skip parsing yaml data from " << status_item.getName() <<
            " on " << status_item.getHwId() <<
            " due to exception: " << e.what());
    }
    catch(YAML::RepresentationException& e)
    {
        ROS_WARN_STREAM(
            "Skip visualizing data as markers for " << status_item.getName() <<
            " on " << status_item.getHwId() <<
            " due to exception: " << e.what());
    }
    // Some exception occurred
    return false;
}
}

class GazeboMarker {
public:
    GazeboMarker(){}

    ~GazeboMarker(){
        // Clean all markers when deleted
        IGNMarker ign_msg;
        ign_msg.set_action(IGNMarker::DELETE_ALL);
        this->_sendMarkerRequest(ign_msg);
    }

    void callbackCymMarker(const ROSMarker&);
    void callbackDiagnosticsAgg(const diagnostic_msgs::DiagnosticArray&);

private:
    void _sendMarkerRequest(const IGNMarker&);

    ignition::transport::Node _ign_node;

};

void GazeboMarker::callbackCymMarker(const ROSMarker& ros_msg)
{
    IGNMarker ign_msg;
    // TODO consider moving conversion and send to another thread
    //  if the list of points is too long (e.g., over 10K)
    ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);

    this->_sendMarkerRequest(ign_msg);
}

void GazeboMarker::_sendMarkerRequest(const IGNMarker& ign_msg)
{
    if (!this->_ign_node.Request("/marker", ign_msg)) {
        ROS_ERROR("Unable to send the marker request to Gazebo.");
        return;
    }
    // else
    return;
}

void GazeboMarker::callbackDiagnosticsAgg(
        const diagnostic_msgs::DiagnosticArray& diag_msg)
{
    // TODO consider moving conversion and send to another thread
    //  if the list of status items or the list of points in items is too long
    //  (e.g., over 10K)
    auto t_begin = ros::Time::now();
    for(auto diag_status : diag_msg.status) {
        diagnostic_aggregator::StatusItem status_item(&diag_status);
        // TODO Generalize to check if the status items are supported markers
        if(status_item.getName() != "reachset")
        {   continue;}
        const auto& format = status_item.getValue("format");
        if(format != "yaml")
        {  // XXX Support other serialization formats possibly with compression. E.g., json
           continue;
        }
        
        IGNMarker ign_msg;
        ros_ign_bridge::convert_ros_to_ign(diag_msg.header,
                                           *ign_msg.mutable_header());
        if( setIGNMarker(ign_msg, status_item) )
        {   // XXX Can Gazebo handle multple marker requests in short time?
            this->_sendMarkerRequest(ign_msg);
        }
    }
    ROS_DEBUG_STREAM("Duration to convert to Gazebo markers: " << (ros::Time::now() - t_begin).toSec());
}


int main(int _argc, char **_argv)
{
    GazeboMarker marker;

    ros::init(_argc, _argv, "cym_marker");
    ros::NodeHandle n;
    ros::Subscriber sub_cym_marker = n.subscribe("cym_marker",
            0,  // XXX Infinite length queue
            &GazeboMarker::callbackCymMarker, &marker);
    // Also check messages and publish Marker messages to Gazabo
    ros::Subscriber sub_diagnostics_agg = n.subscribe("/diagnostics_agg",
            0,  // XXX Infinite length queue
            &GazeboMarker::callbackDiagnosticsAgg, &marker);

    ros::spin();
    return 0;
}
