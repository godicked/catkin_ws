#include <ompl/base/StateSampler.h>

namespace fub
{
namespace planning
{

/**
 * class RoadSampler
 * The RoadSampler extend the CompoundStateSampler and allows
 * to set predicted Road. The idea is to sample states near the road
 * if one is given.
**/

class RoadSampler : public ob::CompoundStateSampler
{
public:
    RoadSampler(const ob::StateSpace *si) : ob::CompoundStateSampler(si)
    {
    }

    void sampleUniform(ob::State *state) override
    {
        if(sample_road_ && road_.size() > 0)
        {
            // std::cout << "sample near" << std::endl;
            double index = rng_.uniformInt(0, road_.size()-1);
            ob::CompoundStateSampler::sampleUniformNear(state, road_[index], width_);
            return;
        }
        else
        {
            // std::cout << "sample" << std::endl;
            ob::CompoundStateSampler::sampleUniform(state);
        }
    }

    void setRoad(std::vector<ob::State *> road, double width)
    {
        road_ = road;
        width_ = width;
        sample_road_ = true;
    }

    void reset()
    {
        road_.clear();
        sample_road_ = false;
    }



protected:
    bool sample_road_ = false;
    double width_;
    std::vector<ob::State *> road_;
    
};

}; // namespace planning
}; // namespace fub