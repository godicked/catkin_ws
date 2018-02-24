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

#include "RRTxStruct.hpp"
#include "Utils.hpp"

using namespace fub::planning;

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
    };
    
    class CostmapValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
    
        CostmapValidityChecker(ompl::base::SpaceInformation *si, costmap_2d::Costmap2D *costmap, double car_lenght, double car_width) : StateValidityChecker(si), costmap_(costmap)
        {
        }


        bool pos_valid(double x, double y) const
        {
            auto s = si_->allocState();
            setX(s, x);
            setY(s, y);
            
            bool result = state_cost(s, costmap_).value() < 100;

            si_->freeState(s);
            return result;
        }
    
        virtual bool isValid(const ompl::base::State *state) const
        {
            double cost = state_cost(state, costmap_).value();
            // std::cout << "cost " << cost << std::endl;

            //  State cannot be valid
            if (cost > 253)
            {
                return false;
            }

            //  Need to test collision with car dimension
            if (cost > 0)
            {
                double yaw = getYaw(state);
                double x = getX(state);
                double y = getY(state);
                
                //  Test collision back 
                double s_x = x + cos(yaw) * (car_length_ / 2);
                double s_y = y + cos(yaw) * (car_length_ / 2);

                bool back = pos_valid(s_x, s_y);

                //  Test collision top
                s_x += car_length_;
                s_y += car_length_;

                bool top = pos_valid(s_x, s_y);

                return back && top;
            }

            //  Cost 0 state is valid
            return true;
        }

    
    private:
        costmap_2d::Costmap2D *costmap_;
        double car_length_;
        double car_width_;
    
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