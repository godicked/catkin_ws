#include <pluginlib/class_list_macros.h>
#include <rrtx/rrtx_planner.hpp>
#include <rrtx/spline_curve.hpp>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>       
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>   
#include <geometry_msgs/Point32.h>              

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt::RRTxPlanner, nav_core::BaseGlobalPlanner)

using namespace std;


void publish_map(ros::Publisher pub, costmap_2d::Costmap2D map)
{
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = "map";
  
  grid.info.width = map.getSizeInCellsX();
  grid.info.height = map.getSizeInCellsY();
  grid.info.resolution = map.getResolution();
  grid.info.origin.position.x = map.getOriginX();
  grid.info.origin.position.y = map.getOriginY();
  
  unsigned char *start = map.getCharMap();
  unsigned char *end = start + map.getSizeInCellsX() * map.getSizeInCellsY();
  grid.data = vector<signed char>(start, end);

  pub.publish(grid);

}

void fill_low_res(costmap_2d::Costmap2D *fullmap, costmap_2d::Costmap2D &low_res, const geometry_msgs::PoseStamped &robot)
{
  double lox = robot.pose.position.x - (low_res.getSizeInMetersX() / 2);
  double loy = robot.pose.position.y - (low_res.getSizeInMetersY() / 2);

  lox = max(lox, fullmap->getOriginX());
  loy = max(loy, fullmap->getOriginY());

  low_res.updateOrigin(lox, loy);

  double min_x, min_y, max_x, max_y;
  min_x = lox;
  min_y = loy;
  max_x = min(low_res.getOriginX() + low_res.getSizeInMetersX(),fullmap->getOriginX() + fullmap->getSizeInMetersX());
  max_y = min(low_res.getOriginY() + low_res.getSizeInMetersY(),fullmap->getOriginY() + fullmap->getSizeInMetersY());

  //ROS_INFO("update cost between: %.1f:%.1f and %.1f:%.1f", min_x, min_y, max_x, max_y);

  for(unsigned int lmx = 0; lmx < low_res.getSizeInCellsX(); lmx++)
  {
      for(unsigned int lmy = 0; lmy < low_res.getSizeInCellsY(); lmy++)
      {
            double wx, wy;
            low_res.mapToWorld(lmx, lmy, wx, wy);
            unsigned int fmx, fmy;
            fullmap->worldToMap(wx, wy, fmx, fmy);

            low_res.setCost(lmx, lmy, fullmap->getCost(fmx, fmy));
      }
  }
}

void publish_polygons(ros::Publisher pub, costmap_converter::PolygonContainerConstPtr polygons, costmap_2d::Costmap2D map)
{
    visualization_msgs::Marker edges;
    edges.header.frame_id  = "map";
    edges.header.stamp     = ros::Time::now();
    edges.ns               = "obstacles";
    edges.id               = 3; 
    edges.action           = visualization_msgs::Marker::ADD;
    edges.type             = visualization_msgs::Marker::LINE_LIST;

    edges.scale.x          = 0.02;
    edges.scale.y          = 0.02;

    edges.color.a          = 1;
    edges.color.g          = 1;

    geometry_msgs::Point p1, p2;
    //ROS_INFO("polygons: %ld", polygons->size());
    for(int i = 0; i < polygons->size(); i++)
    {
        geometry_msgs::Polygon polygon = (*polygons)[i];
        for(int j = 0; j < polygon.points.size(); j++)
        {
            geometry_msgs::Point32 pp1, pp2;
            pp1 = polygon.points[j % polygon.points.size()];
            pp2 = polygon.points[(j+1) % polygon.points.size()];

            p1.x = pp1.x;
            p1.y = pp1.y;

            p2.x = pp2.x;
            p2.y = pp2.y;
            
            edges.points.push_back(p1);
            edges.points.push_back(p2);
        }
    }

    pub.publish(edges);
}

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
  ROS_INFO("init rrtx");
  n = ros::NodeHandle("~/" + name);
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  low_res_costmap = costmap_2d::Costmap2D(100, 100, 0.05, 0, 0);
  path_pub = n.advertise<nav_msgs::Path>("smooth_path", 10);
  stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("low_res", 10);
  poly_pub = n.advertise<visualization_msgs::Marker>("obstacles", 10);
  converter.initialize(n);
  converter.setCostmap2D(&low_res_costmap);
}


bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  //ROS_INFO("make plan");
  
  costmap_converter::PolygonContainerConstPtr polygons = getObstacles(start);
  publishObstacles(polygons);

  if(goalChanged(goal))
  {
      userGoal = goal;
      bool valid = generatePlan(start, goal, plan);
      validGoal = plan.back();
      return valid;
  }
  else
  {
      if(isAtGoal(start))
      {
          stopPlanner();
      }
      else
      {
        fillPath(goal, plan);
      }
      return true;
  }

}

bool RRTxPlanner::generatePlan( const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
{
    costmap_2d::Costmap2D copy = *costmap_;
    rrtx.setCostmap(&copy);
    rrtx.setConstraint(0.4363323129985824,0.3);
    rrtx.init(start.pose, goal.pose);
    rrtx.setMaxDist(5);
    rrtx.grow(1000);
    rrtx.publish(true, true);

    return fillPath(goal, plan);
}

bool RRTxPlanner::fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    RRTx::Path path;
    bool valid = rrtx.getPath(path);
    BSplinePathSmoother smoother;
    path = smoother.curvePath(path, costmap_->getResolution());

    if(!valid) return false;

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

    return true;

}

bool RRTxPlanner::isAtGoal(const geometry_msgs::PoseStamped &robot)
{
    if( abs(robot.pose.position.x - userGoal.pose.position.x) < goal_tolerance &&
        abs(robot.pose.position.y - userGoal.pose.position.y) < goal_tolerance)
    {
        return true;
    }

    if( abs(robot.pose.position.x - validGoal.pose.position.x) < goal_tolerance &&
        abs(robot.pose.position.y - validGoal.pose.position.y) < goal_tolerance)
    {
        ROS_WARN("Could not get better");
        return true;
    }

    return false;
}

void RRTxPlanner::stopPlanner()
{
    ROS_INFO("stop");
    actionlib_msgs::GoalID id;
    stop_pub.publish(id);
}

bool RRTxPlanner::goalChanged(const geometry_msgs::PoseStamped &goal)
{
    if( abs(goal.pose.position.x - userGoal.pose.position.x) < 0.01 &&
        abs(goal.pose.position.y - userGoal.pose.position.y) < 0.01)
    {
        return false;
    }

    return true;
}

costmap_converter::PolygonContainerConstPtr RRTxPlanner::getObstacles(const geometry_msgs::PoseStamped &start)
{
    fill_low_res(costmap_, low_res_costmap, start);
    ros::Time time = ros::Time::now();
    converter.updateCostmap2D();
    converter.compute();
    //std::cout << ros::Time::now() - time << std::endl;
    costmap_converter::PolygonContainerConstPtr polygons = converter.getPolygons();
    return polygons;
}

void RRTxPlanner::publishObstacles(costmap_converter::PolygonContainerConstPtr polygons)
{
    publish_map(map_pub, low_res_costmap);
    publish_polygons(poly_pub, polygons, low_res_costmap);
}

}; //   namespace rrt
