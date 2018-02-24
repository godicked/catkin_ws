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

#include "Utils.hpp"
#include "SE2Config.hpp"
#include "RRTxStruct.hpp"
#include "RoadSampler.hpp"

#include <vector>

#include <tf/tf.h>

namespace ob = ompl::base;

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
                dist += 10;
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
**  The ReedsSheepCostmap accepts a Road as prediction to optimise the sampling process.
*/
class ReedsSheppCostmap : public ob::ReedsSheppStateSpace
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

    void setRoad(std::vector<ob::State *> road, double width)
    {
        // std::cout << "road: " << road_.size() << std::endl;
        road_ = road;
        width_ = width;
        sample_road_ = true;
    }

    // We do not consider the yaw as dimension.
    unsigned int getDimension() const override
    {
        return 2;
    }

    // double getMeasure() const override
    // {
    //     return ob::ReedsSheppStateSpace::getMeasure();
    // }

    ob::StateSamplerPtr allocStateSampler() const override
    {
        auto ss(std::make_shared<RoadSampler>(this));
        if (weightSum_ < std::numeric_limits<double>::epsilon())
            for (unsigned int i = 0; i < componentCount_; ++i)
                ss->addSampler(components_[i]->allocStateSampler(), 1.0);
        else
            for (unsigned int i = 0; i < componentCount_; ++i)
                ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / weightSum_);

        if(sample_road_)
        {
            ss->setRoad(road_, width_);
        }
        return ss;
    }

protected:
    costmap_2d::Costmap2D *costmap_;
    std::vector<ob::State *> road_;
    bool sample_road_ = false;
    double width_;
};


}; // namespace rrt

#endif