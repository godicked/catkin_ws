#include <pluginlib/class_list_macros.h>
#include <rrtx/rrtx_planner.hpp>
#include <rrtx/spline_curve.hpp>

#include <nav_msgs/Path.h>

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
  path_pub = n.advertise<nav_msgs::Path>("smooth_path", 10);
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
    rrtx.publish(true, true);

    fillPath(goal, plan);

    return true;
}

void RRTxPlanner::fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    RRTx::Path path = rrtx.getPath();
    path = curve_path(path, costmap_->getResolution());


    for(auto pose : path)
    {
        //cout << pose.position.x << " " << pose.position.y << endl;

        geometry_msgs::PoseStamped poseStmp;
        poseStmp.header = goal.header;
        poseStmp.pose   = pose;

        plan.push_back(poseStmp);
    }

    nav_msgs::Path spath;
    spath.header = plan.back().header;
    spath.poses = plan;
    
    path_pub.publish(spath);

}

}; //   namespace rrt
