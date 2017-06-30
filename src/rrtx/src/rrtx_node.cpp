#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include "rrtx/rrtx.hpp"

rrt::RRTx *rrtx;

int growSize;
float maxDist;

geometry_msgs::Pose start;

void poseCallback(geometry_msgs::PoseWithCovarianceStamped pose) 
{
    start = pose.pose.pose;
}

void goalCallback(geometry_msgs::PoseStamped goal)
{
    rrtx->setConstraint(0.4363323129985824, 0.26, 2.5089280275926285);
    rrtx->init(start, goal.pose); 
    rrtx->grow(growSize);

    // rrt::RRTx::Path path;
    // rrtx->getPath(path);
    
    rrtx->publish(true, true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtx");
    
    ros::NodeHandle n;
    n.param<int>("grow_size", growSize, 1000);
    n.param<float>("max_dist", maxDist, 2.0);

    ros::Subscriber pose_sub = n.subscribe("initialpose", 10, poseCallback);
    ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 10, goalCallback);


    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    costmap_2d::Costmap2D costmap;

    client.waitForExistence();
    if(client.call(srv)) {
        nav_msgs::OccupancyGrid map = srv.response.map;
        costmap = costmap_2d::Costmap2D(
            map.info.width,
            map.info.height,
            map.info.resolution,
            map.info.origin.position.x,
            map.info.origin.position.y
        );

        for(unsigned int i = 0; i < map.data.size(); i++)
        {
            unsigned int mx, my;
            costmap.indexToCells(i, mx, my);
            costmap.setCost(mx, my, map.data[i]);
        }
    }
    else 
    {
        ROS_WARN("Could not get map from service");
    }

    rrtx = new rrt::RRTx(&costmap);
    rrtx->setMaxDist(maxDist);
    

    ros::spin();

    // ros::Rate rate(1);
    // while(ros::ok())
    // {
        
    //     //cout << "loop" << endl;
    //     rrt.publish(false, true);

    //     rate.sleep();
    // }

    

    return 0;
}

