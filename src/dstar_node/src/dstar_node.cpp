#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <message_filters/subscriber.h>
#include "dstar.hpp"

using namespace std;
using namespace dstar;

DStar *planner;
costmap_t costmap;
geometry_msgs::Point pStart_, pGoal_, pNewStart;
geometry_msgs::PoseStamped sGoal_;
message_filters::Subscriber<nav_msgs::OccupancyGrid> *map_sub;
message_filters::Subscriber<map_msgs::OccupancyGridUpdate> *update_sub;
ros::Subscriber pose_sub;
nav_msgs::OccupancyGridConstPtr grid_;
ros::Publisher update_pub;


unsigned int seq;

bool isInitialized = false;

void startPlanning()
{
  map_sub->unsubscribe();
  update_sub->subscribe();
}

void stopPlanning()
{
  map_sub->subscribe();
  update_sub->unsubscribe();
}

bool isClose(geometry_msgs::Point a, geometry_msgs::Point b)
{
  //return a == b;
  return a.x == b.x && a.y == b.y;
} 

void pose_callback(nav_msgs::OdometryConstPtr odom)
{
  pNewStart.x = odom->pose.pose.position.x;
  pNewStart.y = odom->pose.pose.position.y;
  
  if(abs(pNewStart.x - pGoal_.x) + abs(pNewStart.y - pGoal_.y) < 0.2)
  {
    stopPlanning();
  }
}

void map_callback(nav_msgs::OccupancyGridConstPtr grid)
{
  //ROS_INFO("map_callback");
  grid_ = grid;
  map_sub->subscribe();
}

void update_callback(map_msgs::OccupancyGridUpdateConstPtr gridUpdate)
{
  int width = gridUpdate->width;
  int size = width * gridUpdate->height;
  int origin_x = gridUpdate->x;
  int origin_y = gridUpdate->y;
  bool rebuild = false;

  unsigned int sx, sy;
  costmap.worldToMap(pNewStart.x, pNewStart.y, sx, sy);
  planner->updateStart(sx, sy);

  for(int i = 0; i < size; i++)
  {
    unsigned char cost = gridUpdate->data[i];
    unsigned int my = i / width;
    unsigned int mx = i - (my * width);

    rebuild |= planner->updateCost(mx + origin_x, my + origin_y, cost);
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
  startPlanning();
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
  
  ros::NodeHandle n;
  
  pose_sub = n.subscribe("/robot0/odom", 1, pose_callback);
  
  ros::ServiceServer server = n.advertiseService("get_plan", get_plan_callback);
  
  update_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
  
  map_sub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>();
  map_sub->subscribe(n, "/move_base/global_costmap/costmap", 1);
  map_sub->registerCallback(map_callback);
  
  update_sub = new message_filters::Subscriber<map_msgs::OccupancyGridUpdate>();
  update_sub->subscribe(n, "/move_base/global_costmap/costmap_updates", 10);
  update_sub->unsubscribe();
  update_sub->registerCallback(update_callback);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
