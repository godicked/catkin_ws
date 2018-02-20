#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <string>

using namespace std;

tf::Transform map_to_base;
tf::Transform odom_to_base;

tf::TransformListener *tfl;
tf::TransformBroadcaster *tfb;

string map_frame = "map";
string odom_frame = "odom";
string base_frame = "base_link";


void gpsCallback(nav_msgs::OdometryPtr odom)
{
    auto o = odom->pose.pose.orientation;
    auto p = odom->pose.pose.position;

    tf::Quaternion orientation(o.x, o.y, o.z, o.w);
    tf::Vector3 position(p.x, p.y, 0);

    map_to_base = tf::Transform(orientation, position);
}

bool updateOdom()
{
    tf::StampedTransform odom_pose;
    try
    {
        tfl->lookupTransform(base_frame, odom_frame, ros::Time(0), odom_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    odom_to_base = tf::Transform(odom_pose.getRotation(), odom_pose.getOrigin());
}


void sendTransform()
{
    if(updateOdom())
    {
        tf::Transform base_to_map = map_to_base.inverse();
        tf::Transform map_to_odom = (odom_to_base * base_to_map).inverse();

        ros::Time expiration = ros::Time::now() + ros::Duration(0.1);
        tfb->sendTransform( tf::StampedTransform(map_to_odom, expiration, map_frame, odom_frame) );
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtx_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/visual_gps/odom", 1, gpsCallback);

    tfl = new tf::TransformListener();
    tfb = new tf::TransformBroadcaster();

    ros::Rate r(100);

    while(ros::ok())
    {
        ros::spinOnce();
        sendTransform();
    }

    return 0;
}