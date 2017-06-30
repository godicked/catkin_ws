#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include "dstar.hpp"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace dstar;

timespec diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}


namespace global_planner
{

GlobalPlanner::GlobalPlanner()
{
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  n = new ros::NodeHandle("~/" + name);
  // costmap_ros_ = costmap_ros;
  // costmap_ = costmap_ros_->getCostmap();

  // ros::NodeHandle private_nh("~/" + name);
  // ros::Subscriber sub = private_nh.subscribe("/move_base/global_costmap/costmap_updates", 1, &GlobalPlanner::updateCallback, this);
  
  // originX = costmap_->getOriginX();
  // originY = costmap_->getOriginY();
  // resolution = costmap_->getResolution();

  // ROS_INFO("Initialize Planner plugin");
  // ROS_INFO("costmap origin x: %.1f, y: %.1f", originX, originY);
}



bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_INFO("make plan");
  ros::ServiceClient client = n->serviceClient<nav_msgs::GetPlan>("/get_plan");
  nav_msgs::GetPlan getPlan;
  getPlan.request.start = start;
  getPlan.request.goal = goal;

  
  if(client.call(getPlan))
  {
    ROS_INFO("receive plan from dstar node");
    for(auto pose : getPlan.response.plan.poses)
    {
      plan.push_back(pose);
    }
  }



  // float startX = start.pose.position.x;
  // float startY = start.pose.position.y;

  // float goalX = goal.pose.position.x;
  // float goalY = goal.pose.position.y;

  // ROS_INFO("Make plan");
  // ROS_INFO("start: x: %.1f, y: %.1f", startX, startY);
  // ROS_INFO("goal: x: %.1f, y: %.1f", goalX, goalY);

  // unsigned int startX_ = startX / resolution;
  // unsigned int startY_ = startY / resolution;

  // unsigned int goalX_ = goalX / resolution;
  // unsigned int goalY_ = goalY / resolution;

  // ROS_INFO("start cell: x: %d, y: %d", startX_, startY_);
  // ROS_INFO("start cost: %d", costmap_->getCost(startX_, startY_));
  
  // ROS_INFO("goal cell: x: %d, y: %d", goalX_, goalY_);
  // ROS_INFO("goal cost: %d", costmap_->getCost(goalX_, goalY_));
  
  
  // plan.clear();
  // plan.push_back(start);

  // costmap_2d::Costmap2D costmap = *costmap_;
  // DStar planner = DStar(&costmap);

  // // timespec time1, time2;
  // // /* take current time here */
  // // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
  // ros::Time time1, time2, time3;
  // ros::Duration dtime;
  // time1 = ros::Time::now();
  
  // planner.init(startX_, startY_, goalX_, goalY_);
  // planner.computeShortestPath();
  // path_t path = planner.getPath();

  // time2 = ros::Time::now();
  // dtime = time2 - time1;

  // ROS_INFO("time to generate D lite: %.1f sec", dtime.toSec());
  // dtime = time2 - time1;

  // // clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
  // // cout<<"time to generate best global path by Relaxed A* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;

  // path_t::iterator it;

  // for (it = path.begin(); it != path.end(); it++)
  // {
  //   geometry_msgs::PoseStamped pose = goal;

  //   vertex v = *it;
  //   float x = v.x * resolution;
  //   float y = v.y * resolution;

  //   pose.pose.position.x = x;
  //   pose.pose.position.y = y;
  //   pose.pose.position.z = 0.0;

  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;

  //   plan.push_back(pose);
  //   //ROS_INFO("x: %.1f, y: %.1f", pose.pose.position.x, pose.pose.position.y);
  // }

  return true;
}

};