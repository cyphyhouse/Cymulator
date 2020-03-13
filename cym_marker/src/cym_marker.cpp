/*
 * ROS node providing marker service in Gazeboby sending corresponding requests
 * to Inginition Transport nodes
 *
 * Code modified from
 * https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/marker/marker.cc
 * https://bitbucket.org/osrf/gazebo/src/default/tools/gz_marker.cc
*/

#include <iostream>
#include <string>

#include <ignition/transport.hh>
#include <ignition/msgs.hh>

#include <ros/ros.h>

#include <ros_ign_bridge/convert_marker_interfaces.hpp>

class GazeboMarker {
    typedef cym_marker::Marker ROSMarker;
    typedef ignition::msgs::Marker IGNMarker;
public:
    GazeboMarker(){}

    void sendMarkerRequest(const ROSMarker&);

private:
    ignition::transport::Node _ign_node;
};

void GazeboMarker::sendMarkerRequest(const ROSMarker& ros_msg)
{
    IGNMarker ign_msg;
    ros_ign_bridge::convert_ros_to_ign(ros_msg, ign_msg);
    if (!this->_ign_node.Request("/marker", ign_msg)) {
        std::cerr << "Unable to send marker request to Gazebo." << std::endl;
        return;
    }
    // else
    return;
}


int main(int _argc, char **_argv)
{
    GazeboMarker marker;

    ros::init(_argc, _argv, "cym_marker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cym_marker", 0,  // XXX Infinite length queue
            &GazeboMarker::sendMarkerRequest, &marker);

    ros::spin();
    return 0;
}

