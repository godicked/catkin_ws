#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

// #include <rrtx/spline_curve.hpp>

#include <rrtx/RRTx.hpp>

#include <rrtx/ReedsSheppPublisher.hpp>

#include <rrtx/ReedsSheppConfig.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/MotionValidator.h>

#include <rrtx/SE2Publisher.hpp>

#define PLANNER rrtx

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
double turningRadius;
double solveTime;
bool constraint;
bool limitTime;


typedef ReedsSheppStateSpace::StateType RState;


costmap_2d::Costmap2DROS *costmap;

ProblemDefinitionPtr pdp;
RState *goal_;
RState *start_;

costmap_2d::Costmap2D *cost;

StateSpacePtr ss;
SpaceInformationPtr si;

geometry_msgs::Pose start;

std::shared_ptr<ros::NodeHandle> n;

int test = 0;

void poseCallback(geometry_msgs::PoseWithCovarianceStamped pose) 
{

    tf::Pose tfp;
    
    //  set start yaw
    tf::poseMsgToTF(pose.pose.pose, tfp);
    double yaw = tf::getYaw(tfp.getRotation());
    start_->setYaw(yaw);

    start = pose.pose.pose;
    start_->setX(start.position.x);
    start_->setY(start.position.y);

    // start_->setX(1.3);
    // start_->setY(2.4);

    // unsigned int mx, my;
    // cost->worldToMap(start.position.x, start.position.y, mx, my);
    // ROS_INFO("start at %d, %d", mx, my);

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

    tf::Pose tfp;

    if(test == 0)
    {
        cout << "first solve" << endl;
    //  set goal yaw
    tf::poseMsgToTF(goal.pose, tfp);
    double yaw = tf::getYaw(tfp.getRotation());
    goal_->setYaw(yaw);

    goal_->setX(goal.pose.position.x);
    goal_->setY(goal.pose.position.y);

    // goal_->setX(7.7);
    // goal_->setY(9.2);

    PLANNER->clear();
    pdp->clearSolutionPaths();

    pdp->clearGoal();
    pdp->setGoalState(goal_);

    PLANNER->setProblemDefinition(pdp);
    PLANNER->setRange(maxDist);

    }
    else
    {
        PLANNER->clear();
        pdp->clearSolutionPaths();
        PLANNER->setProblemDefinition(pdp);
        // PLANNER->setRange(5);
        if(test > 5) solveTime = 30;
    }


    const PlannerTerminationCondition ptc([&](){ return PLANNER->numIterations() >= growSize; });

    ros::Time t = ros::Time::now();
    ros::Duration d;
    bool solved = false;


    
    if(limitTime)
    {
        solved = PLANNER->as<Planner>()->solve(solveTime);
    }
    else 
    {
        solved = PLANNER->solve(ptc);
    }
    d = ros::Time::now() - t;

    ROS_INFO("solve took %.2f seconds", d.toSec());
    ROS_INFO("%d iterations", PLANNER->numIterations());
    // cout << "vertices : " << rrts->numVertices() << endl;

    if(test > 0)
    {
        // for(int x = 0; x < 50; x++)
        // {
        //     for(int y = 0; y < 100; y++)
        //     {
        //         cost->setCost(150+x, 90+y, 254);
        //     }
        // }
        // costmap->updateMap();
    
        // auto center = si->allocState()->as<RState>();
        // double wx, wy;
        // cost->mapToWorld(175, 130, wx, wy);
        // center->setXY(wx, wy);
    
        // t = ros::Time::now();
        // // solved = rrtx->updateTree(center, 2.0);
        // // d = ros::Time::now() - t;

        // auto path = pdp->getSolutionPath()->as<PathGeometric>();
        // auto states = path->getStates();
        // cout << "before" << endl;
        // int size = states.size();
    
        // for(int i = 0; i < size-1; i++)
        // {
        //     auto v = states[i];
        //     auto u = states[i+1];
        //     if(!si->checkMotion(v, u))
        //     {
        //         ROS_WARN("obstacle found i: %d",i);
        //         auto center = si->allocState()->as<RState>();
        //         si->getStateSpace()->interpolate(v, u, 0.5, center);
        //         solved = rrtx->updateTree(center, si->distance(v, u));
        //         break;
        //     }
        // }
        // d = ros::Time::now() - t;
        // ROS_INFO("update took %.2f seconds", d.toSec());
        test++;

    }
    
    PlannerData data(si);
    PLANNER->getPlannerData(data);

    if(solved)
    {
        auto path = pdp->getSolutionPath()->as<PathGeometric>();
        auto states = path->getStates(); 

        cout << "solved " << states.size() << endl;

        vector<geometry_msgs::Pose> poses;
        states_to_poses(si, states, poses);

        vector<ompl::base::State *> old_path;
        poses_to_states(si, poses, old_path);
        test++;
        // PLANNER->as<RRTx>()->setSearchPath(old_path, 1);
        // ss.reset( new ReedsSheppCostmap(cost, turningRadius) );
        ss->as<ReedsSheppCostmap>()->setRoad(old_path, 1);

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

void rrtxSetup()
{
    OptimizationObjectivePtr oop;

    if(constraint)
    {
        ss.reset( new ReedsSheppCostmap(cost, turningRadius) );
        // ss->as<ReedsSheppCostmap>()->setRoad(vector<State *>(1), 1);
    }
    else
    {
        ss.reset( new SE2Costmap(cost) );
    }

    si.reset( new SpaceInformation(ss) );

    if(false)
        oop.reset( new SE2OptimizationObjective(si, cost) );
    else
        oop.reset( new ReedsSheppOptimizationObjective(si, cost) );


    StateValidityCheckerPtr svcp( new CostmapValidityChecker(si.get(), cost, 0.30, 0.15) );
    pdp.reset( new ProblemDefinition(si) );

    pdp->setOptimizationObjective( oop );
    si->setStateValidityChecker( svcp );
    
    if(constraint)
        rrt_pub.reset( new ReedsSheppPublisher() );
    else
        rrt_pub.reset( new SE2Publisher() );
    
    rrt_pub->initialize(n.get(), "map", si);

    rrtx.reset( new RRTx(si) );

    goal_ = ss->allocState()->as<RState>();
    start_ = ss->allocState()->as<RState>();
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "rrtx_node");
    
    n.reset( new ros::NodeHandle("~/") );

    n->param<int>("grow_size", growSize, 1000);
    n->param<double>("max_dist", maxDist, 2.0);
    n->param<bool>("constraint", constraint, true);
    n->param<double>("max_steering", max_steering, 0.55);
    n->param<double>("wheelbase", wheelbase, 0.26);
    n->param<double>("turning_radius", turningRadius, 5.0);
    n->param<double>("solve_time", solveTime, 3.0);
    n->param<bool>("limit_time", limitTime, true);

    

    ROS_INFO("init with grow_size: %d, max_dist: %.2f", growSize, maxDist);

    path_pub = n->advertise<nav_msgs::Path>("smooth_path", 10);

    ros::Subscriber pose_sub = n->subscribe("/initialpose", 10, poseCallback);
    ros::Subscriber goal_sub = n->subscribe("/move_base_simple/goal", 10, goalCallback);


    ros::ServiceClient client = n->serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    tf::TransformListener tf(ros::Duration(10));
    costmap = new costmap_2d::Costmap2DROS("costmap", tf);
    cost = costmap->getCostmap();

    rrtxSetup();

    ros::spin();

    return 0;
}

