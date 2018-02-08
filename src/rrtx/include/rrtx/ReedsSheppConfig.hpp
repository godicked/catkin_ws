#ifndef REEDS_SHEPP_CONFIG_HPP
#define REEDS_SHEPP_CONFIG_HPP


#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <rrtx/SE2Config.hpp>

#include <rrtx/RRTxStruct.hpp>

#include <tf/tf.h>

namespace rrt
{
    

/*
**  class ReedsSheppOptimizationObjective
**
**  implements the stateCost and motionCost functions for as Costmap2D
**  used by the RRTx alghorithm
*/

class ReedsSheppOptimizationObjective : public rrt::SE2OptimizationObjective
{
public:

    typedef ompl::base::ReedsSheppStateSpace::ReedsSheppPath Path;

    ReedsSheppOptimizationObjective(ompl::base::SpaceInformationPtr si, costmap_2d::Costmap2D *costmap) : SE2OptimizationObjective(si, costmap)
    {
    }
    
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
    {
        // motion cost is distance in StateSpace
        auto ss = si_->getStateSpace()->as<ompl::base::ReedsSheppStateSpace>();
        Path p = ss->reedsShepp(s1, s2);
        double dist = 0;

        bool direction = p.length_[0] >= 0;

        for(int i = 0; i < 5; i++)
        {
            double d = p.length_[i];
            if(d < 0)
            {
                dist += -2 * d;  
            }
            else
            {
                dist += d;
            }

            bool dir = d >= 0;
            if( direction != dir && d != 0)
            {
                direction = dir;
            // dist += 50;
            }

        }
        // return ompl::base::Cost(p.length());
        return ompl::base::Cost(dist);
    }
};

/*
**  class CostmapStateSpace
**  extends ReedSheepStateSpace
**
**  applys Costmap Boundaries on the StateSpace
*/
class ReedsSheppCostmap : public ompl::base::ReedsSheppStateSpace
{
public:
    ReedsSheppCostmap(costmap_2d::Costmap2D *costmap, double turningRadius = 1.0) : ReedsSheppStateSpace(turningRadius), costmap_(costmap)
    {
        ompl::base::RealVectorBounds bd(2);
        
        // set bounds on x dimension
        bd.setLow(0, costmap->getOriginX());
        bd.setHigh(0, costmap->getOriginX() + costmap->getSizeInMetersX());

        // set bounds on y dimension
        bd.setLow(1, costmap->getOriginY());
        bd.setHigh(1, costmap->getOriginY() + costmap->getSizeInMetersY());

        setBounds(bd);
    }

private:
    costmap_2d::Costmap2D *costmap_;
};


void buildRosPath(ompl::base::SpaceInformationPtr si, std::vector<ompl::base::State *> &path, std::vector<geometry_msgs::Pose> &poses)
{
    std::vector<ompl::base::State *> states;

    int size = path.size() -1;
    for(int i = 0; i < size; i++)
    {
        std::vector<ompl::base::State *> sts;
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

void poses_to_states(ompl::base::SpaceInformationPtr si, std::vector<geometry_msgs::Pose> &poses, std::vector<ompl::base::State *> &states)
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




}; // namespace rrt

#endif