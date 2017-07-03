#ifndef GLOBAL_PLANNER_HPP
#define GLOBAL_PLANNER_HPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>

#include <costmap_converter/costmap_to_lines_ransac.h>
#include <costmap_converter/costmap_to_polygons_concave.h>
#include <costmap_converter/costmap_to_polygons.h>

#include <geometry_msgs/Polygon.h>


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
    costmap_converter::PolygonContainerConstPtr getObstacles(const geometry_msgs::PoseStamped &start);
    void publishObstacles(costmap_converter::PolygonContainerConstPtr polygons);
    

    costmap_2d::Costmap2DROS *costmap_ros_;
    costmap_2d::Costmap2D *costmap_;
    costmap_2d::Costmap2D low_res_costmap;
    ros::NodeHandle n;
    ros::Publisher path_pub;
    ros::Publisher stop_pub;
    ros::Publisher map_pub;
    ros::Publisher poly_pub;
    RRTx rrtx;

    geometry_msgs::PoseStamped validGoal;
    geometry_msgs::PoseStamped userGoal;
    double goal_tolerance = 0.2;

    costmap_converter::CostmapToPolygonsDBSMCCH converter;

    std::vector<geometry_msgs::PoseStamped> last_plan;
};
};
#endif