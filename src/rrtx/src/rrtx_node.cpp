#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <costmap_converter/costmap_to_lines_ransac.h>
#include <costmap_converter/costmap_to_polygons.h>

#include <tf/transform_listener.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <rrtx/spline_curve.hpp>

#include "rrtx/rrtx.hpp"

#include <rrtx/rrtx_publisher.hpp>

#include <rrtx/reeds_shepp_config.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/MotionValidator.h>

#define PLANNER rrts

using namespace ompl::geometric;
using namespace ompl::base;
using namespace costmap_2d;
using namespace rrt;
using namespace std;

std::shared_ptr<RRTstar> rrts;
std::shared_ptr<RRTx> rrtx;

ros::Publisher path_pub;
std::shared_ptr<rrt::RRTxPublisher> rrt_pub;

int growSize;
double maxDist;
double max_steering;
double wheelbase;
bool constraint;

typedef ReedsSheppStateSpace::StateType RState;

ProblemDefinitionPtr pdp;
RState *goal_;
RState *start_;

costmap_2d::Costmap2D *cost;

StateSpacePtr ss;
SpaceInformationPtr si;

geometry_msgs::Pose start;

std::shared_ptr<ros::NodeHandle> n;


void poseCallback(geometry_msgs::PoseWithCovarianceStamped pose) 
{
    start = pose.pose.pose;
    start_->setX(start.position.x);
    start_->setY(start.position.y);

    cout << "done" << endl;
    pdp->clearStartStates();
    pdp->addStartState(start_);
}

void goalCallback(geometry_msgs::PoseStamped goal)
{
    unsigned int mx, my;
    cost->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);

        //if(isOutOfBound(mx, my))
        //    return true;

    ROS_INFO("cost %d", cost->getCost(mx, my));
    //return;


    goal_->setX(goal.pose.position.x);
    goal_->setY(goal.pose.position.y);

    pdp->clearGoal();
    pdp->setGoalState(goal_);

    const PlannerTerminationCondition ptc([&](){ return PLANNER->numIterations() >= growSize; });
    PLANNER->clear();
    pdp->clearSolutionPaths();
    PLANNER->setProblemDefinition(pdp);
    // PLANNER->setMaxDist(maxDist);

    ros::Time t = ros::Time::now();
    auto solved = PLANNER->solve(ptc);
    ros::Duration d = ros::Time::now() - t;

    ROS_INFO("solve took %.2f seconds", d.toSec());

    // cout << "vertices : " << rrts->numVertices() << endl;

    PlannerData data(si);
    PLANNER->getPlannerData(data);

    if(solved)
    {
        auto path = pdp->getSolutionPath()->as<PathGeometric>();
        auto states = path->getStates(); 

        cout << "solved " << states.size() << endl;

        // rrtx->init(pdp); 
        // rrtx->grow(growSize);

        // rrt::RRTx::Path path;
        // if(!rrtx->computePath(path))
        // {
        //     ROS_WARN("no path found!");
        //     return;
        // }
        
        // cout << "computed Path: " << path.size() << endl;

        // rrt::BSplinePathSmoother smoother;
        // // path = smoother.curvePath(path, 0.05);

        vector<geometry_msgs::Pose> poses;
        buildRosPath(si->getStateSpace(), states, poses);

        std::vector<geometry_msgs::PoseStamped> plan;
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
    
    // rrtx->publish(true, true);
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "rrtx_node");
    
    n.reset( new ros::NodeHandle("~/") );
    n->setParam("bool_param", false);

    n->param<int>("grow_size", growSize, 1000);
    n->param<double>("max_dist", maxDist, 2.0);
    n->param<bool>("constraint", constraint, true);
    n->param<double>("max_steering", max_steering, 0.55);
    n->param<double>("wheelbase", wheelbase, 0.26);

    ROS_INFO("init with grow_size: %d, max_dist: %.2f", growSize, maxDist);

    path_pub = n->advertise<nav_msgs::Path>("smooth_path", 10);

    ros::Subscriber pose_sub = n->subscribe("/initialpose", 10, poseCallback);
    ros::Subscriber goal_sub = n->subscribe("/move_base_simple/goal", 10, goalCallback);


    ros::ServiceClient client = n->serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("costmap", tf);

    // client.waitForExistence();
    // if(client.call(srv)) {
    //     nav_msgs::OccupancyGrid map = srv.response.map;
    //     costmap = costmap_2d::StaticLayer(
    //         map.info.width,
    //         map.info.height,
    //         map.info.resolution,
    //         map.info.origin.position.x,
    //         map.info.origin.position.y
    //     );

    //     for(unsigned int i = 0; i < map.data.size(); i++)
    //     {
    //         unsigned int mx, my;
    //         costmap.indexToCells(i, mx, my);
    //         costmap.setCost(mx, my, map.data[i]);
    //     }
    // }
    // else 
    // {
    //     ROS_WARN("Could not get map from service");
    // }

    cost = costmap.getCostmap();

    StateSpacePtr ss( new CostmapStateSpace(cost, 5.0) );

    goal_ = ss->allocState()->as<RState>();
    start_ = ss->allocState()->as<RState>();

    si.reset( new SpaceInformation(ss) );
    MotionValidatorPtr mv( new CostmapMotionValidator(si.get()) );
    StateValidityCheckerPtr svcp( new CostmapValidityChecker(si.get(), cost) );
    OptimizationObjectivePtr oop( new CostmapOptimizationObjective(si, cost) );
    pdp.reset( new ProblemDefinition(si) );
    // MotionValidatorPtr mvp( new ReedsSheppMotionValidator(si.get()));

    pdp->setOptimizationObjective( oop );
    si->setStateValidityChecker( svcp );
    si->setMotionValidator(mv);
    // si->setStateValidityCheckingResolution(0.9);
    
    // rrtx = new RRTx(si);
    // rrtx->setMaxDist(maxDist);
    
    rrt_pub.reset( new RRTxPublisher() );
    rrt_pub->initialize(n.get(), "map", si);

    rrts.reset( new RRTstar(si) );
    rrtx.reset( new RRTx(si) );
    ros::spin();

    return 0;
}

