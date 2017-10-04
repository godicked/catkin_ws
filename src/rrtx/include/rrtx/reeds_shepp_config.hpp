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

#include <rrtx/rrtx_struct.hpp>

namespace rrt
{
    
typedef ompl::base::SE2StateSpace::StateType SE2State;

ompl::base::Cost state_cost(const ompl::base::State *state, costmap_2d::Costmap2D *costmap)
{
    auto st = state->as<ompl::base::ReedsSheppStateSpace::StateType>();

    if(st->getX() < costmap->getOriginX() || st->getX() > costmap->getOriginX() + costmap->getSizeInMetersX() ||
    st->getY() < costmap->getOriginY() || st->getY() > costmap->getOriginY() + costmap->getSizeInMetersY())
    {
        return ompl::base::Cost(255);
    }

    unsigned int mx, my;
    costmap->worldToMap(st->getX(), st->getY(), mx, my);

    return ompl::base::Cost(costmap->getCost(mx ,my));
}

double getX(const ompl::base::State *s)
{
    return s->as<ompl::base::SE2StateSpace::StateType>()->getX();
}

double getY(const ompl::base::State *s)
{
    return s->as<ompl::base::SE2StateSpace::StateType>()->getY();
}




class CostmapValidityChecker : public ompl::base::StateValidityChecker
{
public:

    CostmapValidityChecker(ompl::base::SpaceInformation *si, costmap_2d::Costmap2D *costmap) : StateValidityChecker(si), costmap_(costmap)
    {
    }

    virtual bool isValid(const ompl::base::State *state) const
    {
        double cost = state_cost(state, costmap_).value();
        // std::cout << "cost " << cost << std::endl;
        return cost < 100;
    }

private:
    costmap_2d::Costmap2D *costmap_;

};

/*
**  class CostmapOptimzationObjective
**
**  implements the stateCost and motionCost functions for as Costmap2D
**  used by the RRTx alghorithm
*/

class CostmapOptimizationObjective : public ompl::base::OptimizationObjective
{
public:

    typedef ompl::base::ReedsSheppStateSpace::ReedsSheppPath Path;

    CostmapOptimizationObjective(ompl::base::SpaceInformationPtr si, costmap_2d::Costmap2D *costmap) : OptimizationObjective(si), costmap_(costmap)
    {
    }
    
    virtual ompl::base::Cost stateCost(const ompl::base::State *state) const
    {
        //cost on costmap
        return state_cost(state, costmap_);
    }

    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
    {
        // motion cost is distance in StateSpace
        auto ss = si_->getStateSpace()->as<ompl::base::ReedsSheppStateSpace>();
        Path p = ss->reedsShepp(s1, s2);
        double dist = 0;

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
        }
        // return ompl::base::Cost(p.length());
        return ompl::base::Cost(dist);
    }

private:
    costmap_2d::Costmap2D *costmap_;
};

/*
**  class CostmapStateSpace
**  extends ReedSheepStateSpace
**
**  applys Costmap Boundaries on the StateSpace
*/
class CostmapStateSpace : public ompl::base::ReedsSheppStateSpace
{
public:
    CostmapStateSpace(costmap_2d::Costmap2D *costmap, double turningRadius = 1.0) : ReedsSheppStateSpace(turningRadius), costmap_(costmap)
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

    // re-use ReedsSheppsPath for interpolation, more optimal than StateSpace getMotionStates implementation
    void getMotionStates(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<ompl::base::State *> &states, int nb = 0)
    {

        double t = 0;
        bool firstTime = false;
        ompl::base::ReedsSheppStateSpace::ReedsSheppPath path = reedsShepp(s1, s2);

        if(nb == 0)
        {
            nb = path.length() / costmap_->getResolution();
        }

        for(int i = 0; i < nb; i++)
        {
            ompl::base::State *s = allocState();
            states.push_back(s);
            interpolate(s1, s2, t, firstTime, path, states.back());
            t += 1.0 / (nb);
        }
    }


private:
    costmap_2d::Costmap2D *costmap_;
};

class CostmapMotionValidator : public ompl::base::ReedsSheppMotionValidator
{
public:
    CostmapMotionValidator(ompl::base::SpaceInformation *si) : ReedsSheppMotionValidator(si)
    {
    }

    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
    {
        // std::cout << "checkMotion" << std::endl;
        std::vector<ompl::base::State *> states;
        si_->getStateSpace()->as<CostmapStateSpace>()->getMotionStates(s1, s2, states);

        for(auto s : states)
        {
            if(!si_->isValid(s))
            {
                // std::cout << "Motion not valid!" << std::endl;
                si_->freeStates(states);
                return false;
            }
        }

        si_->freeStates(states);

        return true;
    }
};


void buildRosPath(ompl::base::StateSpacePtr ss, std::vector<ompl::base::State *> &path, std::vector<geometry_msgs::Pose> &poses)
{
    std::vector<ompl::base::State *> states;
    CostmapStateSpace *css = ss->as<CostmapStateSpace>();

    int size = path.size() -1;
    for(int i = 0; i < size; i++)
    {
        css->getMotionStates(path[i], path[i+1], states, 10);
    }

    for(auto s : states)
    {
        geometry_msgs::Pose p;
        p.position.x = getX(s);
        p.position.y = getY(s);

        p.orientation.x = 0;
        p.orientation.y = 0;
        p.orientation.z = 0;
        p.orientation.w = 1;

        poses.push_back(p);
        ss->freeState(s);
    }

}



}; // namespace rrt

#endif