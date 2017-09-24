
#include <costmap_2d/costmap_2d.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>

namespace rrt
{
    
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
        return cost < 250;
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
                dist += -4 * d;  
            }
            else
            {
                dist += d;
            }
        }
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
        si_->getMotionStates(s1, s2, states, si_->distance(s1, s2), false, true);

        for(auto s : states)
        {
            if(!si_->isValid(s))
            {
                // std::cout << "Motion not valid!" << std::endl;
                return false;
            }
        }
        return true;
    }
};


}; // namespace rrt