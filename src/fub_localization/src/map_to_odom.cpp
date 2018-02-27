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

tf::Stamped<tf::Pose> base_pose;


void gpsCallback(nav_msgs::OdometryPtr odom)
{
    auto p = odom->pose.pose.position;

    tf::Pose tfp;
    tf::poseMsgToTF(odom->pose.pose, tfp);

    tf::Quaternion orientation = tfp.getRotation();
    tf::Vector3 position(p.x, p.y, 0);

    map_to_base = tf::Transform(orientation, position);
}

bool updateOdom()
{
    base_pose.stamp_ = ros::Time::now();
    tf::Stamped<tf::Pose> odom_pose;
    try
    {
        tfl->waitForTransform(base_frame, odom_frame, base_pose.stamp_, ros::Duration(0.1));
        tfl->transformPose(odom_frame, base_pose, odom_pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    // odom_to_base = tf::Transform(odom_pose.getRotation(), odom_pose.getOrigin());
    odom_to_base = tf::Transform(odom_pose.getRotation(), odom_pose.getOrigin());
}


void sendTransform()
{
    if(updateOdom())
    {
        tf::Transform base_to_map = map_to_base.inverse();
        tf::Transform map_to_odom = (odom_to_base * base_to_map).inverse();

        ros::Time expiration = ros::Time::now() + ros::Duration(0.0);
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

    base_pose = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), ros::Time::now(), base_frame);

    ros::Rate r(100);

    while(ros::ok())
    {
        ros::spinOnce();
        sendTransform();
    }

    return 0;
}