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


#include <google/protobuf/text_format.h>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>


class GazeboMarker {
public:
    GazeboMarker(){}

    void sendMarkerRequest(const std_msgs::String&);

private:
    ignition::transport::Node _ign_node;
};

void GazeboMarker::sendMarkerRequest(const std_msgs::String& _msg)
{
    if(_msg.data.empty())
        return;

    ignition::msgs::Marker msg;
    if (!google::protobuf::TextFormat::ParseFromString(_msg.data, &msg)) {
        std::cerr << "Invalid string message: " << _msg << std::endl;
        return;
    }

    if (!this->_ign_node.Request("/marker", msg)) {
        std::cerr << "Unable to send marker request to Gazebo." << std::endl;
        return;
    }
    // else
    return;
}


int main(int _argc, char **_argv)
{
    GazeboMarker marker;

    ros::init(_argc, _argv, "gazebo_marker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gazebo_marker", 0,  // XXX Infinite length queue
            &GazeboMarker::sendMarkerRequest, &marker);

    ros::spin();
    return 0;
}

