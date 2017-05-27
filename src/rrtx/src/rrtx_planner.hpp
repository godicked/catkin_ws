#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include "rrtx.hpp"

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace rrt
{

class RRTxPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    RRTxPlanner();
    RRTxPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);

  private:
    bool generatePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

    void fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
    

    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    ros::NodeHandle n;
    RRTx rrtx;
};
};
#endif