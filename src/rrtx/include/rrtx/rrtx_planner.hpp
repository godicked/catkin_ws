#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <geometry_msgs/Polygon.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/State.h>

#include <rrtx/reeds_shepp_config.hpp>
#include <rrtx/rrtx_publisher.hpp>

#include "rrtx.hpp"

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

    bool fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
    bool isAtGoal(const geometry_msgs::PoseStamped &robot);
    void stopPlanner();
    bool goalChanged(const geometry_msgs::PoseStamped &goal);
    // costmap_converter::PolygonContainerConstPtr getObstacles(const geometry_msgs::PoseStamped &start);
    // void publishObstacles(costmap_converter::PolygonContainerConstPtr polygons);
    

    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    costmap_2d::Costmap2D low_res_costmap;
    boost::shared_ptr<costmap_2d::Layer> static_layer;
    //costmap_2d::Layer obstacle_layer;

    rrt::RRTxPublisher *rrt_pub;
    
    ros::NodeHandle n;
    ros::Publisher path_pub;
    ros::Publisher stop_pub;
    ros::Publisher map_pub;
    ros::Publisher poly_pub;
    std::shared_ptr<RRTx> rrtx_;

    geometry_msgs::PoseStamped validGoal;
    geometry_msgs::PoseStamped userGoal;
    double goal_tolerance = 0.2;
    bool solved_;

    std::vector<geometry_msgs::PoseStamped> last_plan;

    ompl::base::ProblemDefinitionPtr pdp_;
    ompl::base::SpaceInformationPtr si;

    SE2State *start_;
    SE2State *goal_;

};
}; // namespace rrt
#endif