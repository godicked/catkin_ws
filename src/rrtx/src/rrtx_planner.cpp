#include <pluginlib/class_list_macros.h>
#include <rrtx/rrtx_planner.hpp>
#include <rrtx/spline_curve.hpp>
#include <actionlib_msgs/GoalID.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>       
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Point.h>   
#include <geometry_msgs/Point32.h>          

// #include <rrtx/reeds_shepp_config.hpp>
#include <ompl/base/MotionValidator.h>
#include <ompl/geometric/PathGeometric.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt::RRTxPlanner, nav_core::BaseGlobalPlanner)

using namespace ompl::base;
using namespace costmap_2d;
using namespace std;


// void publish_map(ros::Publisher pub, costmap_2d::Costmap2D map)
// {
//   nav_msgs::OccupancyGrid grid;
//   grid.header.stamp = ros::Time::now();
//   grid.header.frame_id = "map";
  
//   grid.info.width = map.getSizeInCellsX();
//   grid.info.height = map.getSizeInCellsY();
//   grid.info.resolution = map.getResolution();
//   grid.info.origin.position.x = map.getOriginX();
//   grid.info.origin.position.y = map.getOriginY();
  
//   unsigned char *start = map.getCharMap();
//   unsigned char *end = start + map.getSizeInCellsX() * map.getSizeInCellsY();
//   grid.data = vector<signed char>(start, end);

//   pub.publish(grid);

// }

// void activate_static_map(bool active)
// {
//     dynamic_reconfigure::ReconfigureRequest srv_req;
//     dynamic_reconfigure::ReconfigureResponse srv_resp;
//     dynamic_reconfigure::BoolParameter bool_param;
//     dynamic_reconfigure::Config conf;

//     bool_param.name = "enabled";
//     bool_param.value = active;
//     conf.bools.push_back(bool_param);

//     srv_req.config = conf;

//     ros::service::call("/move_base/global_costmap/static_layer/set_parameters", srv_req, srv_resp);
// }



// void fill_low_res(costmap_2d::Costmap2D *fullmap, costmap_2d::Costmap2D &low_res, const geometry_msgs::PoseStamped &robot)
// {

//   double lox = robot.pose.position.x - (low_res.getSizeInMetersX() / 2);
//   double loy = robot.pose.position.y - (low_res.getSizeInMetersY() / 2);

//   lox = max(lox, fullmap->getOriginX());
//   loy = max(loy, fullmap->getOriginY());

//   low_res.updateOrigin(lox, loy);

//   double min_x, min_y, max_x, max_y;
//   min_x = lox;
//   min_y = loy;
//   max_x = min(low_res.getOriginX() + low_res.getSizeInMetersX(),fullmap->getOriginX() + fullmap->getSizeInMetersX());
//   max_y = min(low_res.getOriginY() + low_res.getSizeInMetersY(),fullmap->getOriginY() + fullmap->getSizeInMetersY());

//   //ROS_INFO("update cost between: %.1f:%.1f and %.1f:%.1f", min_x, min_y, max_x, max_y);

//   for(unsigned int lmx = 0; lmx < low_res.getSizeInCellsX(); lmx++)
//   {
//       for(unsigned int lmy = 0; lmy < low_res.getSizeInCellsY(); lmy++)
//       {
//             double wx, wy;
//             low_res.mapToWorld(lmx, lmy, wx, wy);
//             unsigned int fmx, fmy;
//             fullmap->worldToMap(wx, wy, fmx, fmy);
//             unsigned char cost = fullmap->getCost(fmx, fmy);
//             if( cost > 150 )
//                 cost = 254;
            
//             low_res.setCost(lmx, lmy, cost);
//       }
//   }
// }

// void publish_polygons(ros::Publisher pub, costmap_converter::PolygonContainerConstPtr polygons, costmap_2d::Costmap2D map)
// {
//     visualization_msgs::Marker edges;
//     edges.header.frame_id  = "map";
//     edges.header.stamp     = ros::Time::now();
//     edges.ns               = "obstacles";
//     edges.id               = 3; 
//     edges.action           = visualization_msgs::Marker::ADD;
//     edges.type             = visualization_msgs::Marker::LINE_LIST;

//     edges.scale.x          = 0.02;
//     edges.scale.y          = 0.02;

//     edges.color.a          = 1;
//     edges.color.g          = 1;

//     geometry_msgs::Point p1, p2;
//     //ROS_INFO("polygons: %ld", polygons->size());
//     for(int i = 0; i < polygons->size(); i++)
//     {
//         geometry_msgs::Polygon polygon = (*polygons)[i];
//         for(int j = 0; j < polygon.points.size(); j++)
//         {
//             geometry_msgs::Point32 pp1, pp2;
//             pp1 = polygon.points[j % polygon.points.size()];
//             pp2 = polygon.points[(j+1) % polygon.points.size()];

//             p1.x = pp1.x;
//             p1.y = pp1.y;

//             p2.x = pp2.x;
//             p2.y = pp2.y;
            
//             edges.points.push_back(p1);
//             edges.points.push_back(p2);
//         }
//     }

//     pub.publish(edges);
// }

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
//   low_res_costmap = costmap_2d::Costmap2D(200, 200, 0.05, 0, 0);
  path_pub = n.advertise<nav_msgs::Path>("smooth_path", 10);
  stop_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

//   map_pub = n.advertise<nav_msgs::OccupancyGrid>("low_res", 10);
//   poly_pub = n.advertise<visualization_msgs::Marker>("obstacles", 10);
//   converter.initialize(n);
//   converter.setCostmap2D(&low_res_costmap);

//   costmap_2d::LayeredCostmap *layeredMap = costmap_ros_->getLayeredCostmap();
//   std::vector< boost::shared_ptr< costmap_2d::Layer > > *layers = layeredMap->getPlugins();
//   for(auto layer : *layers)
//   {
//       //ROS_INFO("candidate %s", layer->getName().c_str());
//       if(layer->getName().find("inflation_layer") != string::npos)
//       {
//           //ROS_INFO("found %s", layer->getName().c_str());
//           static_layer = layer;
//       }
//   }

    StateSpacePtr ss( new CostmapStateSpace(costmap_, 5.0) );

    goal_ = ss->allocState()->as<SE2State>();
    start_ = ss->allocState()->as<SE2State>();

    si.reset( new SpaceInformation(ss) );
    MotionValidatorPtr mv( new CostmapMotionValidator(si.get()) );
    StateValidityCheckerPtr svcp( new CostmapValidityChecker(si.get(), costmap_) );
    OptimizationObjectivePtr oop( new CostmapOptimizationObjective(si, costmap_) );
    pdp_.reset( new ProblemDefinition(si) );
    // MotionValidatorPtr mvp( new ReedsSheppMotionValidator(si.get()));

    pdp_->setOptimizationObjective( oop );
    si->setStateValidityChecker( svcp );
    si->setMotionValidator(mv);
    // si->setStateValidityCheckingResolution(0.9);
    
    rrtx_.reset( new RRTx(si) );

    rrt_pub = new RRTxPublisher();
    rrt_pub->initialize(&n, "map", si);
}


bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  //ROS_INFO("make plan");
  
  //costmap_converter::PolygonContainerConstPtr polygons = getObstacles(start);
  //publishObstacles(polygons);

//   fill_low_res(costmap_, low_res_costmap, start);

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
        // rrtx_->updateTree(
        //     low_res_costmap.getOriginX(), 
        //     low_res_costmap.getOriginY(),
        //     low_res_costmap.getSizeInMetersX(),
        //     low_res_costmap.getSizeInMetersY()
        // );
        // rrtx_->updateRobot(start.pose);

        fillPath(goal, plan);
      }
      return true;
  }

}

bool RRTxPlanner::generatePlan( const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
{
    // activate_static_map(true);
    // costmap_ros_->updateMap();

    //  set X,Y coordinate
    start_->setXY(start.pose.position.x, start.pose.position.y);
    goal_->setXY(goal.pose.position.x, goal.pose.position.y);

    tf::Pose pose;

    //  set start yaw
    tf::poseMsgToTF(start.pose, pose);
    double yaw = tf::getYaw(pose.getRotation());
    start_->setYaw(yaw);

    //  set goal yaw
    tf::poseMsgToTF(goal.pose, pose);
    yaw = tf::getYaw(pose.getRotation());
    goal_->setYaw(yaw);
    
    pdp_->clearStartStates();
    pdp_->clearGoal();

    pdp_->addStartState(start_);
    pdp_->setGoalState(goal_);

    rrtx_->setProblemDefinition(pdp_);
    rrtx_->setRange(4);
    solved_ = rrtx_->solve(4.0);
    //activate_static_map(false);

    return fillPath(goal, plan);
}

bool RRTxPlanner::fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    PlannerData data(si);
    rrtx_->getPlannerData(data);

    if(solved_)
    {
        auto path = pdp_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        auto states = path->getStates(); 

        cout << "solved " << states.size() << endl;

        vector<geometry_msgs::Pose> poses;
        buildRosPath(si->getStateSpace(), states, poses);

        for(auto pose : poses)
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
    else
    {
        cout << "not solved" << endl;
    }
    
    rrt_pub->publish(data);

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

// costmap_converter::PolygonContainerConstPtr RRTxPlanner::getObstacles(const geometry_msgs::PoseStamped &start)
// {
//     fill_low_res(costmap_, low_res_costmap, start);
//     ros::Time time = ros::Time::now();
//     converter.updateCostmap2D();
//     converter.compute();
//     //std::cout << ros::Time::now() - time << std::endl;
//     costmap_converter::PolygonContainerConstPtr polygons = converter.getPolygons();
//     return polygons;
// }

// void RRTxPlanner::publishObstacles(costmap_converter::PolygonContainerConstPtr polygons)
// {
//     publish_map(map_pub, low_res_costmap);
//     publish_polygons(poly_pub, polygons, low_res_costmap);
// }

}; //   namespace rrt
