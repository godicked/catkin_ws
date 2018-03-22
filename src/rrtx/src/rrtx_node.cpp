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
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/MotionValidator.h>

#include <rrtx/SE2Publisher.hpp>
#include <rrtx/Utils.hpp>
#include <boost/thread.hpp>

#include <global_planner/planner_core.h>

#define PLANNER rrtx

using namespace ompl::geometric;
using namespace ompl::base;
using namespace costmap_2d;
using namespace rrt;
using namespace std;


void publishSearch(bool solved);
void updateTree(double frequency);

std::shared_ptr<RRTx> rrtx;
std::shared_ptr<RRTstar> rrts;
std::shared_ptr<global_planner::GlobalPlanner> grid_planner;

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
bool prediction;
double road_width;
int goal_bias;

string map_frame = "map";
string odom_frame = "odom";
string base_frame = "base_link";

double update_frequecy;

tf::Stamped<tf::Pose> base_pose;


typedef ReedsSheppStateSpace::StateType RState;


costmap_2d::Costmap2DROS *costmapRos;
costmap_2d::Costmap2D::mutex_t *costmap_mutex;

std::vector<State *> road;

ProblemDefinitionPtr pdp;
RState *goal_;
RState *start_;

bool goal_is_set = false;
bool start_is_set = false;

costmap_2d::Costmap2D *costmap;

StateSpacePtr ss;
SpaceInformationPtr si;

StateSpacePtr se2StateSpace;

geometry_msgs::Pose start;

std::shared_ptr<ros::NodeHandle> n;

geometry_msgs::PoseStamped goal_pose;

void sendStop()
{
    nav_msgs::Path path;
    path.header = goal_pose.header;
    path.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped poseStmp;
    poseStmp.header = goal_pose.header;
    poseStmp.header.stamp = ros::Time::now();
    
    // tf_to_pose(map_pose, poseStmp.pose);
    // path.poses.push_back(poseStmp);

    path_pub.publish(path);

    ROS_INFO("Stop planner");
}


void compute_simple_path(RState *start, RState *goal, std::vector<State *> &path)
{
    geometry_msgs::PoseStamped st_start, st_goal;
    state_to_pose(start, st_start.pose);
    state_to_pose(goal, st_goal.pose);

    st_start.header.frame_id = "map";
    st_goal.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> plan;
    grid_planner->makePlan(st_start, st_goal, plan);

    // nav_msgs::Path spath;
    // spath.header = plan.back().header;
    // spath.poses = plan;
    // spath.header.stamp = ros::Time::now();
    // path_pub.publish(spath);

    std::vector<geometry_msgs::Pose> pose_plan;
    for(auto p : plan)
    {
        pose_plan.push_back(p.pose);
    }

    compute_orientations(pose_plan);
    poses_to_states(si, pose_plan, path);
}

/**
 * Pose update for Rviz
**/
bool first_time = true;
void poseCallback(geometry_msgs::PoseWithCovarianceStamped pose) 
{
    pose_to_state(pose.pose.pose, start_);

    pdp->clearStartStates();
    pdp->addStartState(start_);

    start_is_set = true;

    if(goal_is_set)
    {
        costmap_mutex->lock();
        // bool solved = PLANNER->updateRobot(start_);
        // publishSearch(solved);
        costmap_mutex->unlock();

    }
    
    ROS_INFO("Position updated");
}



/**
 * User Goal callback
**/  
void goalCallback(geometry_msgs::PoseStamped goal)
{

    if(!start_is_set)
    {
        ROS_WARN("No start position given. Abort planning");
        return;
    }

    // stop controller when planning a new path
    if(goal_is_set)
    {
        sendStop();
    }

    pose_to_state(goal.pose, goal_);

    if(prediction)
    {
        si->freeStates(road);
        compute_simple_path(start_, goal_, road);
        ss->as<ReedsSheppCostmap>()->setRoad(road, road_width);
    }


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

    costmap_mutex->lock();
    solved = PLANNER->as<Planner>()->solve(solveTime);
    costmap_mutex->unlock();

    d = ros::Time::now() - t;

    ROS_INFO("Sampling during %.2f seconds", d.toSec());
    // ROS_INFO("%d iterations", PLANNER->numIterations());
    
    publishSearch(solved);

    goal_is_set = true;
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
            // ROS_ERROR("%s", ex.what());
            rate.sleep();
            continue;
        }

        // convert transform to ompl state
        ompl::base::State *robot = si->allocState();
        tf_to_state(map_pose, robot);

        bool update = false;
        // if no start was specified or the robot position changed enough
        // set start in the problem definition
        if(!start_is_set || si->distance(start_, robot) > 0.1)
        {
            si->copyState(start_, robot);
            pdp->clearStartStates();
            pdp->addStartState(start_);
            start_is_set = true;
            update = true;
        }

        // free state !!
        si->freeState(robot);


        if(!goal_is_set)
        {
            rate.sleep();
            continue;
        }
 
        
        double dist_to_goal = se2StateSpace->distance(start_, goal_);
        // If robot near goal publish gaol as path to stop the local planner
        if(dist_to_goal < 0.3)
        {
            // update_thread_shutdown = true;
            goal_is_set = false;
            
            sendStop();

            rate.sleep();
            continue;
        }

        costmap_mutex->lock();
        if(update)
        {
            ROS_INFO("Distance to goal: %.2f", dist_to_goal);
            // PLANNER->updateRobot(start_);
        }
        bool solved = PLANNER->verifyPath();
        costmap_mutex->unlock();
        publishSearch(solved);
        // ROS_INFO("Update path");  

        rate.sleep();
    }


}


void publishSearch(bool solved)
{
    // return;
    PlannerData data(si);
    PLANNER->getPlannerData(data);

    if(solved)
    {
        auto path = pdp->getSolutionPath()->as<PathGeometric>();
        auto states = path->getStates(); 

        // ROS_INFO("Solved! Path has %d vertices", (int)states.size());

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
        ss.reset( new ReedsSheppCostmap(costmap, turningRadius) );
    }
    else
    {
        ss.reset( new SE2Costmap(costmap) );
    }

    se2StateSpace.reset( new ompl::base::SE2StateSpace );

    si.reset( new SpaceInformation(ss) );

    if(!constraint)
        oop.reset( new SE2OptimizationObjective(si, costmap) );
    else
        oop.reset( new ReedsSheppOptimizationObjective(si, costmap) );


    // oop->setCostToGoHeuristic(oop->motionCost);

    StateValidityCheckerPtr svcp( new CostmapValidityChecker(si.get(), costmap, 0.30, 0.15) );
    pdp.reset( new ProblemDefinition(si) );

    pdp->setOptimizationObjective( oop );
    si->setStateValidityChecker( svcp );
    
    if(constraint)
        rrt_pub.reset( new ReedsSheppPublisher() );
    else
        rrt_pub.reset( new SE2Publisher() );
    
    rrt_pub->initialize(n.get(), "map", si);

    rrtx.reset( new RRTx(si) );
    rrts.reset( new RRTstar(si));

    goal_ = ss->allocState()->as<RState>();
    start_ = ss->allocState()->as<RState>();

    rrtx->setGoalBias(goal_bias);
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
    n->param<bool>("prediction", prediction, false);
    n->param<double>("road_width", road_width, 1.0);
    n->param<int>("goal_bias", goal_bias, 20);

    // n->param<bool>("limit_time", limitTime, true);

    base_pose = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), ros::Time::now(), base_frame);

    ROS_INFO("init max_dist: %.2f", maxDist);

    path_pub = n->advertise<nav_msgs::Path>("smooth_path", 10);

    ros::Subscriber pose_sub = n->subscribe("/initialpose", 10, poseCallback);
    ros::Subscriber goal_sub = n->subscribe("/move_base_simple/goal", 10, goalCallback);


    ros::ServiceClient client = n->serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;

    tfl = new tf::TransformListener(ros::Duration(10));
    costmapRos = new costmap_2d::Costmap2DROS("costmap", *tfl);
    costmap = costmapRos->getCostmap();
    costmap_mutex = costmap->getMutex();

    grid_planner.reset( new global_planner::GlobalPlanner("grid_planner", costmap, "map") );

    rrtxSetup();

    if(update_frequecy > 0)
    {
        update_thread = new boost::thread(boost::bind(updateTree, update_frequecy));
    }

    ros::spin();
    si->freeStates(road);

    return 0;
}

