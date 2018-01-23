#ifndef SE2_PUBLISHER_HPP
#define SE2_PUBLISHER_HPP

#include <rrtx/RRTxPublisher.hpp>
#include <rrtx/SE2Config.hpp>

namespace rrt
{

class SE2Publisher : public RRTxPublisher
{

protected:

    virtual void trajectoryPose(const State *v, const State *u, std::vector<geometry_msgs::Pose> &poses)
    {
        double dist = si_->distance(v, u);
        auto ss = si_->getStateSpace()->as<ompl::base::SE2StateSpace>();
        double t = 1 / (dist * 4);


        for(double i = 0; i < 1+t; i += t)
        {
            // std::cout << "i: " << i << std::endl;
            i = std::min(i, 1.0);

            geometry_msgs::Pose p;
            auto *state = si_->allocState()->as<SE2StateSpace::StateType>();

            ss->interpolate(v, u, i, state);
            p.position.x = state->getX();
            p.position.y = state->getY();
            poses.push_back(p);

            if(i == 1)
                break;
        }
    }    

};  //  class ReedsSheppPublisher

};  //  namespace rrt


#endif