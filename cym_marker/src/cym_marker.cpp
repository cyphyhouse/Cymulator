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
#include <cmath>
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
    const double DELTA = 1.0;  // meter
    typedef std::bitset<3> Vertex3d;
    const std::array<const Vertex3d, 8> vertice = {
        0b000, 0b011, 0b110, 0b101,
        0b001, 0b010, 0b100, 0b111};
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
        for(size_t i=0; i<3; ++i)
        {
            IGNPoint p1;
            {
                const double x_d = std::copysign(DELTA, corner[ targets[i][0] ].x()-p0.x());
                const double y_d = std::copysign(DELTA, corner[ targets[i][1] ].y()-p0.y());
                const double z_d = std::copysign(DELTA, corner[ targets[i][2] ].z()-p0.z());
                p1.set_x(p0.x() + x_d);
                p1.set_y(p0.y() + y_d);
                p1.set_z(p0.z() + z_d);
            }

            IGNPoint p2;
            {
                size_t j = (i+1)%3;
                const double x_d = std::copysign(DELTA, corner[ targets[j][0] ].x()-p0.x());
                const double y_d = std::copysign(DELTA, corner[ targets[j][1] ].y()-p0.y());
                const double z_d = std::copysign(DELTA, corner[ targets[j][2] ].z()-p0.z());
                p2.set_x(p0.x() + x_d);
                p2.set_y(p0.y() + y_d);
                p2.set_z(p0.z() + z_d);
            }

            *msg.add_point() = p0;  // This will copy points
            *msg.add_point() = p1;
            *msg.add_point() = p2;
            // Add three points in different order so we can see from the other side
            *msg.add_point() = p0;
            *msg.add_point() = p2;
            *msg.add_point() = p1;
        }

    }
    return;
}

void addBoxFaces(IGNMarker& msg, const std::array<IGNPoint, 2>& corner)
{
    typedef std::bitset<3> Vertex3d;

    // Construct 6 faces of the rectangle with 4*3 = 12 triangles
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
        for(size_t i=0; i<3; ++i)
        {
            // Add source point by copying
            *msg.add_point() = p0;
            // Create and directly set target point
            IGNPoint& p1 = *msg.add_point(); 
            p1.set_x(corner[ targets[i][0] ].x());
            p1.set_y(corner[ targets[i][1] ].y());
            p1.set_z(corner[ targets[i][2] ].z());

            IGNPoint& p2 = *msg.add_point();
            size_t j = (i+1)%3;
            p2.set_x(corner[ targets[j][0] ].x());
            p2.set_y(corner[ targets[j][1] ].y());
            p2.set_z(corner[ targets[j][2] ].z());
        }
    }
    return;
}

void addCylinder(IGNMarker& msg, const std::array<IGNPoint, 2>& corner)
{
    const size_t N = 32;  // Number of points to approximate ellipses
    typedef std::array<double, 3> PointT;

    const double r_x = std::abs(corner[0].x() - corner[1].x()) / 2;
    const double r_y = std::abs(corner[0].y() - corner[1].y()) / 2;
    const double r_z = std::abs(corner[0].z() - corner[1].z()) / 2;

    const double xy_ratio = std::max(r_x, r_y)/std::min(r_x, r_y);
    const double yz_ratio = std::max(r_y, r_z)/std::min(r_y, r_z);
    const double zx_ratio = std::max(r_z, r_x)/std::min(r_z, r_x);

    std::vector<PointT> top_point_vec, bot_point_vec, side_point_vec;
    top_point_vec.reserve(3*(N+1));
    bot_point_vec.reserve(3*(N+1));
    side_point_vec.reserve(6*(N+1));
    if (xy_ratio <= yz_ratio && xy_ratio <= zx_ratio || xy_ratio <= 3.0) { // prefer aligning cylinder with z-axis
        // Create points for top, bottom, and side of cylinder
        // Notice the order of points beind added
        for(size_t i=0; i<N; ++i) {
            const double theta0 = (double(i)/N)*2*M_PI;
            const double theta1 = (double(i+1)/N)*2*M_PI;
            const double x0 = r_x * std::cos(theta0);
            const double y0 = r_y * std::sin(theta0);
            const double x1 = r_x * std::cos(theta1);
            const double y1 = r_y * std::sin(theta1);

            top_point_vec.push_back({0.0, 0.0, r_z});
            top_point_vec.push_back({x0, y0, r_z});
            top_point_vec.push_back({x1, y1, r_z});

            side_point_vec.push_back({x0, y0, r_z});
            side_point_vec.push_back({x0, y0, -r_z});
            side_point_vec.push_back({x1, y1, -r_z});

            side_point_vec.push_back({x1, y1, r_z});
            side_point_vec.push_back({x0, y0, r_z});
            side_point_vec.push_back({x1, y1, -r_z});

            bot_point_vec.push_back({0.0, 0.0, -r_z});
            bot_point_vec.push_back({x1, y1, -r_z});
            bot_point_vec.push_back({x0, y0, -r_z});
        }
    }
    else if (yz_ratio <= zx_ratio) {
        for(size_t i=0; i<N; ++i) {
            const double theta0 = (double(i)/N)*2*M_PI;
            const double theta1 = (double(i+1)/N)*2*M_PI;
            const double y0 = r_y * std::cos(theta0);
            const double z0 = r_z * std::sin(theta0);
            const double y1 = r_y * std::cos(theta1);
            const double z1 = r_z * std::sin(theta1);

            top_point_vec.push_back({r_x, 0.0, 0.0});
            top_point_vec.push_back({r_x, y0, z0});
            top_point_vec.push_back({r_x, y1, z1});

            side_point_vec.push_back({r_x, y0, z0});
            side_point_vec.push_back({-r_x, y0, z0});
            side_point_vec.push_back({-r_x, y1, z1});

            side_point_vec.push_back({r_x, y1, z1});
            side_point_vec.push_back({r_x, y0, z0});
            side_point_vec.push_back({-r_x, y1, z1});

            bot_point_vec.push_back({-r_x, 0.0, 0.0});
            bot_point_vec.push_back({-r_x, y1, z1});
            bot_point_vec.push_back({-r_x, y0, z0});
        }
    }
    else {
        for(size_t i=0; i<N; ++i) {
            const double theta0 = (double(i)/N)*2*M_PI;
            const double theta1 = (double(i+1)/N)*2*M_PI;
            const double z0 = r_z * std::cos(theta0);
            const double x0 = r_x * std::sin(theta0);
            const double z1 = r_z * std::cos(theta1);
            const double x1 = r_x * std::sin(theta1);

            top_point_vec.push_back({0.0, r_y, 0.0});
            top_point_vec.push_back({x0, r_y, z0});
            top_point_vec.push_back({x1, r_y, z1});

            side_point_vec.push_back({x0, r_y, z0});
            side_point_vec.push_back({x0, -r_y, z0});
            side_point_vec.push_back({x1, -r_y, z1});

            side_point_vec.push_back({x1, r_y, z1});
            side_point_vec.push_back({x0, r_y, z0});
            side_point_vec.push_back({x1, -r_y, z1});

            bot_point_vec.push_back({0.0, -r_y, 0.0});
            bot_point_vec.push_back({x1, -r_y, z1});
            bot_point_vec.push_back({x0, -r_y, z0});
        }
    }

    const double center_x = (corner[0].x() + corner[1].x()) / 2;
    const double center_y = (corner[0].y() + corner[1].y()) / 2;
    const double center_z = (corner[0].z() + corner[1].z()) / 2;
    for(auto p: top_point_vec) {
        IGNPoint& ign_p = *msg.add_point();
        ign_p.set_x(center_x + p[0]);
        ign_p.set_y(center_y + p[1]);
        ign_p.set_z(center_z + p[2]);
    }
    for(auto p: side_point_vec) {
        IGNPoint& ign_p = *msg.add_point();
        ign_p.set_x(center_x + p[0]);
        ign_p.set_y(center_y + p[1]);
        ign_p.set_z(center_z + p[2]);
    }
    for(auto p: bot_point_vec) {
        IGNPoint& ign_p = *msg.add_point();
        ign_p.set_x(center_x + p[0]);
        ign_p.set_y(center_y + p[1]);
        ign_p.set_z(center_z + p[2]);
    }
}

void addHollowEllipsoid(IGNMarker& msg, const std::array<IGNPoint, 2>& corner)
{
    const size_t N = 32;  // Number of points to approximate ellipses
    const double STEP = 0.375; // meter
    IGNPoint center;
    center.set_x((corner[0].x() + corner[1].x()) / 2);
    center.set_y((corner[0].y() + corner[1].y()) / 2);
    center.set_z((corner[0].z() + corner[1].z()) / 2);

    const double r_x = std::abs(corner[0].x() - corner[1].x()) / 2;
    const double r_y = std::abs(corner[0].y() - corner[1].y()) / 2;
    const double r_z = std::abs(corner[0].z() - corner[1].z()) / 2;

    const double xy_ratio = std::max(r_x, r_y)/std::min(r_x, r_y);
    const double yz_ratio = std::max(r_y, r_z)/std::min(r_y, r_z);
    const double zx_ratio = std::max(r_z, r_x)/std::min(r_z, r_x);

    const std::string draw = (xy_ratio <= yz_ratio && xy_ratio <= zx_ratio)? "xy" :
                             (yz_ratio <= zx_ratio)? "yz" : "zx";

    if(draw != "xy")
    {   /* skip drawing very thin ellipses */ }
    else {
        for(double step_z=r_z; step_z>=-r_z; step_z-=STEP) {
            // Create ellipise for xy-plane
            IGNPoint p;
            p.set_x(center.x() + r_x);
            p.set_y(center.y());
            p.set_z(center.z() + step_z);
            for(size_t i=1; i<=N; ++i) {
                msg.add_point()->CopyFrom(p);  // Append the previous point to start the line segment
                const double theta = (double(i)/N)*2*M_PI;
                const double x = r_x * std::cos(theta);
                const double y = r_y * std::sin(theta);

                p.set_x(center.x() + x);
                p.set_y(center.y() + y);
                p.set_z(center.z() + step_z);

                msg.add_point()->CopyFrom(p);  // Append current point to end the line segment
            }
        }
    }

    if(draw != "yz")
    {   /* skip */ }
    else {
        for(double step_x=r_x; step_x>=-r_x; step_x-=STEP) {
            // Create ellipise for yz-plane
            IGNPoint p;
            p.set_x(center.x() + step_x);
            p.set_y(center.y() + r_y);
            p.set_z(center.z());
            for(size_t i=1; i<=N; ++i) {
                msg.add_point()->CopyFrom(p);  // Append the previous point to start the line segment
                const double theta =(double(i)/N)*2*M_PI;
                const double y = r_y * std::cos(theta);
                const double z = r_z * std::sin(theta);

                p.set_x(center.x() + step_x);
                p.set_y(center.y() + y);
                p.set_z(center.z() + z);

                msg.add_point()->CopyFrom(p);  // Append current point to end the line segment
            }
        }
    }

    if(draw != "zx")
    {   /* skip */ }
    else {
        for(double step_y=r_y; step_y>=-r_y; step_y-=STEP) {
            // Create ellipise for zx-plane
            IGNPoint p;
            p.set_x(center.x());
            p.set_y(center.y() + step_y);
            p.set_z(center.z() + r_z);
            for(size_t i=1; i<=N; ++i) {
                msg.add_point()->CopyFrom(p);  // Append the previous point to start the line segment
                const double theta = (double(i)/N)*2*M_PI;
                const double z = r_z * std::cos(theta);
                const double x = r_x * std::sin(theta);

                p.set_x(center.x() + x);
                p.set_y(center.y() + step_y);
                p.set_z(center.z() + z);

                msg.add_point()->CopyFrom(p);  // Append current point to end the line segment
            }
        }
    }
}

void addBoundingCylinders(IGNMarker& msg,
                          const YAML::Node& box_seq, bool hollow)
{
    assert(box_seq.IsSequence());
    for(auto box : box_seq)
    {
        std::array<IGNPoint, 2> p;  // p[0] is p_min and p[1] is p_max
        p[0].set_x(box[0][0].as<double>());
        p[0].set_y(box[0][1].as<double>());
        p[0].set_z(box[0][2].as<double>());
        p[1].set_x(box[1][0].as<double>());
        p[1].set_y(box[1][1].as<double>());
        p[1].set_z(box[1][2].as<double>());

        if(hollow)
            addHollowBox(msg, p);
        else
            addCylinder(msg, p);
    }
}


void addReachSet(IGNMarker& msg, const YAML::Node& state_list)
{
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
    }
}

void addContract(IGNMarker& msg, const YAML::Node& set_repr, bool hollow)
{
    if(!set_repr.IsMap())
        return;
    for(auto it = set_repr.begin(); it != set_repr.end(); ++it)
    {
        const YAML::Node& set_type = it->first;
        if(set_type.as<std::string>() == "BoxesMap")
        {
            if(!it->second.IsSequence()){
                ROS_WARN_STREAM("Unexpected " << it->first << ":" << it->second << std::endl);
                continue;
            }
            addBoundingCylinders(msg, it->second, hollow);
        }
        else if(set_type.as<std::string>() == "BoxesMap")
        {
            if(!it->second.IsSequence()){
                ROS_WARN_STREAM("Unexpected " << it->first << ":" << it->second << std::endl);
                continue;
            }
            for(auto box : it->second)
            {
                std::array<IGNPoint, 2> p;
                p[0].set_x(box[0][0].as<double>());
                p[0].set_y(box[0][1].as<double>());
                p[0].set_z(box[0][2].as<double>());
                p[1].set_x(box[1][0].as<double>());
                p[1].set_y(box[1][1].as<double>());
                p[1].set_z(box[1][2].as<double>());

                if(hollow)
                    addHollowBox(msg, p);
                else
                    addBoxFaces(msg, p);
            }
        }
        else if(set_type.as<std::string>() == "ContiguousUnion")
        {
            if(!it->second.IsSequence())
            {
                ROS_WARN_STREAM("Unexpected " << it->first << ":" << it->second << std::endl);
                continue;
            }
            const auto& t_r_list = it->second;
            // TODO binary search to find iterator
            auto it=t_r_list.begin();
            YAML::Node prev_region(YAML::NodeType::Map);
            for(; it!= t_r_list.end(); ++it)
            {
                auto op = *it;
                double t = op[0].as<double>();
                if(t >= ros::Time::now().toSec())
                    break;
                prev_region = op[1];
            }

            addContract(msg, prev_region, hollow);
            for(size_t i=0; it!= t_r_list.end() && i<15; ++it, ++i)
            {
                addContract(msg, (*it)[1], hollow);
            }
            for(; it!= t_r_list.end(); ++it)
            {
                addContract(msg, (*it)[1], true);
            }
        }
        else if(set_type.as<std::string>() == "Union")
        {
            if(!it->second.IsSequence())
            {
                ROS_WARN_STREAM("Unexpected " << it->first << ":" << it->second << std::endl);
                continue;
            }
            for(auto op : it-> second)
                addContract(msg, op, hollow);
        }
        else
        {
            ROS_WARN_STREAM("Unexpected set type " << it->first << std::endl);
            // TODO skip for now
        }
    }
}


bool setIGNMarker(IGNMarker& msg,
                  const diagnostic_aggregator::StatusItem& status_item)
{
    try 
    {
        YAML::Node yaml_node = YAML::Load(status_item.getValue("data"));
        if(status_item.getName() == "reachset")
        {
            addReachSet(msg, yaml_node);
            return true;
        }
        if (status_item.getName() == "contract")
        {
            addContract(msg, yaml_node, false);
            return true;
        }
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


std::vector<IGNMarker>
buildIGNMarkers(const diagnostic_aggregator::StatusItem& status_item)
{
    std::vector<IGNMarker> vec;
    vec.push_back(IGNMarker());
    IGNMarker& msg = vec.back();
    msg.set_type(IGNMarker::TRIANGLE_LIST);
    msg.set_ns(status_item.getHwId());
    msg.set_id(1);  // Any non zero id to denote updating the same marker instead of creating new markers
    auto& mat = *msg.mutable_material();
    auto& script = *mat.mutable_script();

    const std::string& material = status_item.getValue("material");
    if(material != "")
        script.set_name(material);
    else
        script.set_name("Gazebo/GreenTransparentOverlay");

    setIGNMarker(msg, status_item);
    return vec;
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
        if(status_item.getName() != "reachset" && status_item.getName() != "contract")
        {   continue;}
        const auto& format = status_item.getValue("format");
        if(format != "yaml")
        {  // XXX Support other serialization formats possibly with compression. E.g., json
           continue;
        }

        std::vector<IGNMarker> ign_msg_vec = buildIGNMarkers(status_item);
        if( !ign_msg_vec.empty() )
        {   // XXX Can Gazebo handle multple marker requests in short time?
            for(auto ign_msg: ign_msg_vec) {
                ros_ign_bridge::convert_ros_to_ign(diag_msg.header,
                                                   *ign_msg.mutable_header());
                this->_sendMarkerRequest(ign_msg);
            }
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
