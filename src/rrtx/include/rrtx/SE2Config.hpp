#ifndef SE2_CONFIG_HPP
#define SE2_CONFIG_HPP


#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Pose.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <rrtx/RRTxStruct.hpp>

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

    double getYaw(const ompl::base::State *s)
    {
        return s->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
    }
    
    double getX(const ompl::base::State *s)
    {
        return s->as<ompl::base::SE2StateSpace::StateType>()->getX();
    }
    
    double getY(const ompl::base::State *s)
    {
        return s->as<ompl::base::SE2StateSpace::StateType>()->getY();
    }
    
    void setX(ompl::base::State *s, double x)
    {
        s->as<ompl::base::SE2StateSpace::StateType>()->setX(x);
    }
    
    void setY(ompl::base::State *s, double y)
    {
        s->as<ompl::base::SE2StateSpace::StateType>()->setY(y);
    }
    
    class SE2Costmap : public ompl::base::SE2StateSpace
    {
    public:
        SE2Costmap(costmap_2d::Costmap2D *costmap)
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


    class SE2OptimizationObjective : public ompl::base::OptimizationObjective
    {
    public:
    
        typedef ompl::base::ReedsSheppStateSpace::ReedsSheppPath Path;
    
        SE2OptimizationObjective(ompl::base::SpaceInformationPtr si, costmap_2d::Costmap2D *costmap) : OptimizationObjective(si), costmap_(costmap)
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
            return ompl::base::Cost(si_->distance(s1, s2));
        }

    protected:
        costmap_2d::Costmap2D *costmap_;
    };



};  // namespace


#endif