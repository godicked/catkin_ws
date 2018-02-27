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
#include <rrtx/Utils.hpp>
#include <boost/thread.hpp>

#define PLANNER rrtx

using namespace ompl::geometric;
using namespace ompl::base;
using namespace costmap_2d;
using namespace rrt;
using namespace std;


void publishSearch(bool solved);
void updateTree(double frequency);

std::shared_ptr<RRTx> rrtx;

ros::Publisher path_pub;
std::shared_ptr<rrt::RRTxPublisher> rrt_pub;

boost::thread *update_thread = NULL;

tf::TransformListener *tfl;

int growSize;
double maxDist;
double max_steering;
double wheelbase;
double turningRadius;
double solveTime;
bool constraint;
bool limitTime;
bool save_last;
bool publish_tree;
double obstacle_resolution;

string map_frame = "map";
string odom_frame = "odom";
string base_frame = "base_link";

double update_frequecy;

tf::Stamped<tf::Pose> base_pose;


typedef ReedsSheppStateSpace::StateType RState;


costmap_2d::Costmap2DROS *costmap;

ProblemDefinitionPtr pdp;
RState *goal_;
RState *start_;

costmap_2d::Costmap2D *cost;

StateSpacePtr ss;
SpaceInformationPtr si;

StateSpacePtr se2StateSpace;

geometry_msgs::Pose start;

std::shared_ptr<ros::NodeHandle> n;

geometry_msgs::PoseStamped goal_pose;

/**
 * Pose update for Rviz
**/
bool first_time = true;
void poseCallback(geometry_msgs::PoseWithCovarianceStamped pose) 
{
    pose_to_state(pose.pose.pose, start_);

    pdp->clearStartStates();
    pdp->addStartState(start_);

    if(first_time)
    {
        first_time = false;
    }
    else
    {
        bool solved = PLANNER->updateRobot(start_);
        publishSearch(solved);    
    }
    
    ROS_INFO("Position updated");
}

/**
 * User Goal callback
**/  
void goalCallback(geometry_msgs::PoseStamped goal)
{
    pose_to_state(goal.pose, goal_);

    goal_pose = goal;

    PLANNER->clear();
    pdp->clearSolutionPaths();

    pdp->clearGoal();
    pdp->setGoalState(goal_);

    PLANNER->setProblemDefinition(pdp);
    PLANNER->setRange(maxDist);
    ss->setLongestValidSegmentFraction(obstacle_resolution);

    ros::Time t = ros::Time::now();
    ros::Duration d;
    bool solved = false;

    solved = PLANNER->as<Planner>()->solve(solveTime);

    d = ros::Time::now() - t;

    ROS_INFO("Sampling during %.2f seconds", d.toSec());
    ROS_INFO("%d iterations", PLANNER->numIterations());
    
    publishSearch(solved);

    if(update_thread == NULL)
    {
        update_thread = new boost::thread(boost::bind(updateTree, update_frequecy));
    }
}

bool update_thread_shutdown = false;
void updateTree(double frequency)
{
    ros::Rate rate(frequency);

    while(ros::ok() && !update_thread_shutdown)
    {
        base_pose.stamp_ = ros::Time::now();
        base_pose.frame_id_ = base_frame;
        tf::Stamped<tf::Pose> map_pose;
        try
        {
            tfl->waitForTransform(base_frame, map_frame, base_pose.stamp_, ros::Duration(0.1));
            tfl->transformPose(map_frame, base_pose, map_pose);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            continue;
        }

        ompl::base::State *robot = si->allocState();
        tf_to_state(map_pose, robot);

        if(se2StateSpace->distance(robot, goal_) < 0.2)
        {
            si->freeState(robot);
            update_thread_shutdown = true;


            nav_msgs::Path path;
            path.header = goal_pose.header;
            path.header.stamp = ros::Time::now();
        
            geometry_msgs::PoseStamped poseStmp;
            poseStmp.header = goal_pose.header;
            poseStmp.header.stamp = ros::Time::now();
            
            tf_to_pose(map_pose, poseStmp.pose);
            
        
            path.poses.push_back(poseStmp);
        
            path_pub.publish(path);

            return;
        }

        // Donc update for small position changes
        if(si->distance(start_, robot) < 0.2)
        {
            si->freeState(robot);
            continue;
        }

        si->copyState(start_, robot);
        si->freeState(robot);

        pdp->clearStartStates();
        pdp->addStartState(start_);
 
        bool solved = PLANNER->updateRobot(start_);
        publishSearch(solved);    
        
        ROS_INFO("Position updated");

        rate.sleep();
    }


}

void publishSearch(bool solved)
{
    PlannerData data(si);
    PLANNER->getPlannerData(data);

    if(solved)
    {
        auto path = pdp->getSolutionPath()->as<PathGeometric>();
        auto states = path->getStates(); 

        ROS_INFO("Solved!");

        vector<geometry_msgs::Pose> poses;
        states_to_poses(si, states, poses);

        std::vector<geometry_msgs::PoseStamped> plan;
        for(auto pose : poses)
        {
            geometry_msgs::PoseStamped poseStmp;
            poseStmp.header = goal_pose.header;
            poseStmp.header.stamp = ros::Time::now();
            poseStmp.pose   = pose;

            plan.push_back(poseStmp);
        }
        
        nav_msgs::Path spath;
        spath.header = plan.back().header;
        spath.poses = plan;
        spath.header.stamp = ros::Time::now();
        path_pub.publish(spath);
    }
    else
    {
        ROS_WARN("No path found");
        nav_msgs::Path path;
        path.header = goal_pose.header;
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
    }
    if(publish_tree)
    {
        rrt_pub->publish(data);
    }
}

void rrtxSetup()
{
    OptimizationObjectivePtr oop;

    if(constraint)
    {
        ss.reset( new ReedsSheppCostmap(cost, turningRadius) );
    }
    else
    {
        ss.reset( new SE2Costmap(cost) );
    }

    se2StateSpace.reset( new ompl::base::SE2StateSpace );

    si.reset( new SpaceInformation(ss) );

    if(!constraint)
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

    // n->param<int>("grow_size", growSize, 1000);
    n->param<double>("max_dist", maxDist, 2.0);
    n->param<bool>("constraint", constraint, true);
    n->param<double>("max_steering", max_steering, 0.55);
    n->param<double>("wheelbase", wheelbase, 0.26);
    n->param<double>("turning_radius", turningRadius, 5.0);
    n->param<double>("solve_time", solveTime, 3.0);
    n->param<string>("map_frame", map_frame, "map");
    n->param<string>("base_frame", base_frame, "base_link");
    n->param<double>("update_frequency", update_frequecy, 1/5.0);
    n->param<bool>("publish_tree", publish_tree, true);
    n->param<double>("obstalce_resolution", obstacle_resolution, 0.005);
    // n->param<bool>("limit_time", limitTime, true);

    base_pose = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), ros::Time::now(), base_frame);

    ROS_INFO("init max_dist: %.2f", maxDist);

    path_pub = n->advertise<nav_msgs::Path>("smooth_path", 10);

    ros::Subscriber pose_sub = n->subscribe("/initialpose", 10, poseCallback);
    ros::Subscriber goal_sub = n->subscribe("/move_base_simple/goal", 10, goalCallback);


    ros::ServiceClient client = n->serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    tfl = new tf::TransformListener(ros::Duration(10));
    costmap = new costmap_2d::Costmap2DROS("costmap", *tfl);
    cost = costmap->getCostmap();

    rrtxSetup();

    ros::spin();

    return 0;
}

