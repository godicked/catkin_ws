#include <pluginlib/class_list_macros.h>
#include <rrtx/RRTxPlanner.hpp>
// #include <rrtx/spline_curve.hpp>
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

    n.param<double>("max_dist", max_distance_, 4.0);
    n.param<double>("turning_radius", turning_radius_, 0.55);
    n.param<double>("car_length", car_length_, 0.3);
    n.param<double>("car_width", car_width_, 0.15);
    n.param<double>("solve_time", solve_time_, 3.0);
    n.param<double>("longest_valid_segment", longest_valid_segment_, 0.1);

    StateSpacePtr ss(new ReedsSheppCostmap(costmap_, turning_radius_));
    ss->setLongestValidSegmentFraction(longest_valid_segment_);

    goal_ = ss->allocState()->as<SE2State>();
    start_ = ss->allocState()->as<SE2State>();

    si_.reset(new SpaceInformation(ss));
    // MotionValidatorPtr mv( new ReedsSheppCostmapMotionValidator(si_.get()) );
    StateValidityCheckerPtr svcp(new CostmapValidityChecker(si_.get(), costmap_, car_length_, car_width_));
    OptimizationObjectivePtr oop(new ReedsSheppOptimizationObjective(si_, costmap_));
    pdp_.reset(new ProblemDefinition(si_));
    // MotionValidatorPtr mvp( new ReedsSheppMotionValidator(si_.get()));

    pdp_->setOptimizationObjective(oop);
    si_->setStateValidityChecker(svcp);
    // si_->setMotionValidator(mv);
    // si_->setStateValidityCheckingResolution(0.9);

    rrtx_.reset(new RRTx(si_));

    rrt_pub.reset(new ReedsSheppPublisher());
    rrt_pub->initialize(&n, "map", si_);
}

bool RRTxPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    ROS_INFO("make plan");

    if (goalChanged(goal))
    {
        userGoal = goal;
        bool valid = generatePlan(start, goal, plan);
        validGoal = plan.back();
        return valid;
    }
    else
    {
        if (isAtGoal(start))
        {
            stopPlanner();
        }
        else
        {
            updatePath();
            fillPath(goal, plan);
        }
        return true;
    }
}

bool RRTxPlanner::generatePlan(const geometry_msgs::PoseStamped &start,
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
    rrtx_->setRange(max_distance_);
    rrtx_->clear();
    solved_ = rrtx_->solve(solve_time_);
    //activate_static_map(false);

    return fillPath(goal, plan);
}

//  Update the optimal path if collision is detected
//  In order to save time only edges near collision are tested
void RRTxPlanner::updatePath()
{

    ROS_INFO("Update Path");

    if (!solved_)
    {
        return;
    }

    auto path = pdp_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    auto states = path->getStates();

    for (int i = 0; i < states.size() - 1; i++)
    {
        auto v = states[i];
        auto u = states[i + 1];
        if (!si_->checkMotion(v, u))
        {
            auto center = si_->allocState();
            si_->getStateSpace()->interpolate(v, u, 0.5, center);
            rrtx_->updateTree(center, si_->distance(v, u));
            path = pdp_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
            states = path->getStates();
            i = -1;
            si_->freeState(center);
        }
    }
}

bool RRTxPlanner::fillPath(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    PlannerData data(si_);
    rrtx_->getPlannerData(data);

    if (solved_)
    {
        auto path = pdp_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        auto states = path->getStates();

        cout << "solved " << states.size() << endl;

        vector<geometry_msgs::Pose> poses;
        states_to_poses(si_, states, poses);

        for (auto pose : poses)
        {
            //cout << pose.position.x << " " << pose.position.y << endl;

            geometry_msgs::PoseStamped poseStmp;
            poseStmp.header = goal.header;
            poseStmp.pose = pose;

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
    if (abs(robot.pose.position.x - userGoal.pose.position.x) < goal_tolerance &&
        abs(robot.pose.position.y - userGoal.pose.position.y) < goal_tolerance)
    {
        return true;
    }

    if (abs(robot.pose.position.x - validGoal.pose.position.x) < goal_tolerance &&
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
    if (abs(goal.pose.position.x - userGoal.pose.position.x) < 0.01 &&
        abs(goal.pose.position.y - userGoal.pose.position.y) < 0.01)
    {
        return false;
    }

    return true;
}

}; //   namespace rrt
