#ifndef REEDS_SHEPP_PUBLISHER_HPP
#define REEDS_SHEPP_PUBLISHER_HPP

#include <rrtx/RRTxPublisher.hpp>
#include <rrtx/ReedsSheppConfig.hpp>

namespace rrt
{

class ReedsSheppPublisher : public RRTxPublisher
{

protected:

    virtual void trajectoryPose(const State *v, const State *u, std::vector<geometry_msgs::Pose> &poses)
    {
        double dist = si_->distance(v, u);
        auto ss = si_->getStateSpace()->as<ReedsSheppStateSpace>();
        double t = 1 / (dist * 4);
        ReedsSheppStateSpace::ReedsSheppPath path;
        bool firstTime = true;

        // std::cout << "dist " << dist << std::endl;
        // std::cout << "t " << t << std::endl;


        for(double i = 0; i < 1+t; i += t)
        {
            // std::cout << "i: " << i << std::endl;
            i = std::min(i, 1.0);

            geometry_msgs::Pose p;
            ReedsSheppStateSpace::StateType *state = ss->allocState()->as<ReedsSheppStateSpace::StateType>();

            ss->interpolate(v, u, i, firstTime, path, state);
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