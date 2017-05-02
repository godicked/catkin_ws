#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include "dstar.hpp"

using namespace std;
using namespace dstar;

DStar *planner;
costmap_t costmap;
geometry_msgs::Point pStart_, pGoal_, pPose;
geometry_msgs::PoseStamped sGoal_;
ros::NodeHandle *n;
ros::Subscriber map_sub;
ros::Subscriber update_sub;
ros::Subscriber pose_sub;
nav_msgs::OccupancyGridConstPtr grid_;
ros::Publisher update_pub;


unsigned int seq;

bool isInitialized = false;

bool isClose(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return a.x == b.x && a.y == b.y;
} 

void refresh_map();

void pose_callback(nav_msgs::OdometryConstPtr odom)
{
  pPose.x = odom->pose.pose.position.x;
  pPose.y = odom->pose.pose.position.y;
}

void map_callback(nav_msgs::OccupancyGridConstPtr grid)
{
  //ROS_INFO("map_callback");
  grid_ = grid;
  refresh_map();
}

void refresh_map()
{
  map_sub.shutdown();
  map_sub = n->subscribe("/move_base/global_costmap/costmap", 1, map_callback);
}

void update_callback(map_msgs::OccupancyGridUpdateConstPtr gridUpdate)
{
  ROS_INFO("update");
  int width = gridUpdate->width;
  int size = width * gridUpdate->height;
  int origin_x = gridUpdate->x;
  int origin_y = gridUpdate->y;
  bool rebuild = false;

  unsigned int sx, sy;
  costmap.worldToMap(pPose.x, pPose.y, sx, sy);
  planner->updateStart(sx, sy);

  for(int i = 0; i < size; i++)
  {
    unsigned char cost = gridUpdate->data[i];
    unsigned int my = (i / width) + origin_y;
    unsigned int mx = i - (my * width) + origin_x;

    if(mx < costmap.getSizeInCellsX() &&
       my < costmap.getSizeInCellsY())
    {   
      rebuild |= planner->updateCost(mx, my, cost);
    }
  }
  if(rebuild)
  {
    planner->computeShortestPath();
    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.header = sGoal_.header;
    actionGoal.goal.target_pose = sGoal_;
    update_pub.publish(actionGoal);
  }
    
}

void init()
{
  ROS_INFO("init");
  unsigned int size_x = grid_->info.width;
  unsigned int size_y = grid_->info.height;
  unsigned int size = size_x * size_y;
  float resolution = grid_->info.resolution;
  double origin_x = grid_->info.origin.position.x;
  double origin_y = grid_->info.origin.position.y;
  vertex start, goal;
  
  costmap.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  costmap.worldToMap(pStart_.x, pStart_.y, start.x, start.y);
  costmap.worldToMap(pGoal_.x, pGoal_.y, goal.x, goal.y);

  for(int i = 0; i < size; i++)
  {
    unsigned int mx, my;
    costmap.indexToCells(i, mx, my);
    costmap.setCost(mx, my, grid_->data[i]);
  }

  delete planner;
  planner = new DStar(costmap);
  planner->init(start.x, start.y, goal.x, goal.y);

  ROS_INFO("init DStar start: %d,%d", start.x, start.y);
  update_sub = n->subscribe("/move_base/global_costmap/costmap_updates", 10, update_callback);
  isInitialized = true;
}

bool sendPath(nav_msgs::GetPlanResponse &res)
{

  planner->computeShortestPath();
  path_t path = planner->getPath();

  res.plan.header.seq = seq;
  res.plan.header.stamp = ros::Time::now();
  res.plan.header.frame_id = sGoal_.header.frame_id;

  for(auto v : path)
  {
    geometry_msgs::PoseStamped spose = sGoal_;
    costmap.mapToWorld(v.x, v.y, spose.pose.position.x, spose.pose.position.y);
    res.plan.poses.push_back(spose);
  }
  seq++;
  return true;
}

bool get_plan_callback(nav_msgs::GetPlanRequest &req, nav_msgs::GetPlanResponse &res)
{
  ROS_INFO("get_plan_callback");

  geometry_msgs::Point pStart, pGoal;
  pStart = req.start.pose.position;
  pGoal = req.goal.pose.position;
  sGoal_ = req.goal;

  if(!isClose(pGoal, pGoal_))
  {
    isInitialized = false;
  }
  if(!isInitialized)
  {
    ROS_INFO("get costmap");
    pGoal_ = pGoal;
    pStart_ = pStart;

    init();
  }
  
  return sendPath(res);
}
//dstar_node::Path p;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "dstar_node");
  
  n = new ros::NodeHandle();
  pose_sub = n->subscribe("/robot0/odom", 1, pose_callback);
  ros::ServiceServer server = n->advertiseService("get_plan", get_plan_callback);
  update_pub = n->advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
  refresh_map();

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
