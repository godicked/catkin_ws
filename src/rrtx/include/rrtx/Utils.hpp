#ifndef PATH_UTILS_HPP
#define PATH_UTILS_HPP

#include <geometry_msgs/Pose.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

/**
 * Utility functions for ROS related datatypes
 * Conversion between geomtry_msg::Pose and ob::State
**/

namespace fub
{
namespace planning
{

namespace ob = ompl::base;

/**
 * Accesors for OMPL SE2State
 * Every SE2State has a position X,Y and a Yaw
**/

double getYaw(const ob::State *s)
{
    return s->as<ob::SE2StateSpace::StateType>()->getYaw();
}

double getX(const ob::State *s)
{
    return s->as<ob::SE2StateSpace::StateType>()->getX();
}

double getY(const ob::State *s)
{
    return s->as<ob::SE2StateSpace::StateType>()->getY();
}

void setX(ob::State *s, double x)
{
    s->as<ob::SE2StateSpace::StateType>()->setX(x);
}

void setY(ob::State *s, double y)
{
    s->as<ob::SE2StateSpace::StateType>()->setY(y);
}

void setYaw(ob::State *s, double yaw)
{
    s->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
}

/**
 * geometry_msg::Pose and ompl::base::State conversions
**/

void states_to_poses(ob::SpaceInformationPtr si, std::vector<ob::State *> &path, std::vector<geometry_msgs::Pose> &poses)
{
    std::vector<ob::State *> states;

    int size = path.size() -1;
    for(int i = 0; i < size; i++)
    {
        std::vector<ob::State *> sts;
        si->getMotionStates(path[i], path[i+1], sts, 10, true, true);
        states.insert(states.end(), sts.begin(), sts.end());
    }

    for(auto s : states)
    {
        geometry_msgs::Pose p;
        p.position.x = getX(s);
        p.position.y = getY(s);
        p.position.z = 0.5;

        double yaw = getYaw(s);
        auto q = tf::createQuaternionFromRPY(0, 0, yaw);

        p.orientation.x = q[0];
        p.orientation.y = q[1];
        p.orientation.z = q[2];
        p.orientation.w = q[3];

        poses.push_back(p);
        
    }

    si->freeStates(states);

}

void poses_to_states(ob::SpaceInformationPtr si, std::vector<geometry_msgs::Pose> &poses, std::vector<ob::State *> &states)
{
    int size = poses.size();
    for(int i = 0; i < size; i++)
    {
        auto state = si->allocState();
        auto pose = poses[i];

        setX(state, pose.position.x);
        setY(state, pose.position.y);

        tf::Pose tfp;
        tf::poseMsgToTF(pose, tfp);
        double yaw = tf::getYaw(tfp.getRotation());
        setYaw(state, yaw);

        states.push_back(state);
    }
}



}; // namespace planning
}; // namespace fub

#endif