#include <pluginlib/class_list_macros.h>
#include "rrtx_planner.hpp"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt::RRTxPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace rrt
{

RRTxPlanner::RRTxPlanner()
{
}

RRTxPlanner::RRTxPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros);
}

void RRTxPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  n = ros::NodeHandle("~/" + name);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
}



bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_INFO("make plan");
  
  generatePlan(start, goal, plan);
  return true;
}

bool RRTxPlanner::generatePlan( const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
{
    costmap_2d::Costmap2D copy = *costmap_;
    rrtx.setCostmap(&copy);

    rrtx.init(start.pose, goal.pose);
    rrtx.setMaxDist(5);
    rrtx.grow(1000);
    rrtx.publish(false, true);

    fillPath(goal, plan);

    return true;
}

void RRTxPlanner::fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    RRTx::Path path = rrtx.getPath();
    
    for(auto pose : path)
    {
        geometry_msgs::PoseStamped poseStmp;
        poseStmp.header = goal.header;
        poseStmp.pose   = pose;

        plan.push_back(poseStmp);
    }

}

}; //   namespace rrt