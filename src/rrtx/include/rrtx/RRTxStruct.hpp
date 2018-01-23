#ifndef RRTX_STRUCT_HPP
#define RRTX_STRUCT_HPP

#include <limits>
#include <stdlib.h>
#include <vector>
#include <boost/unordered_map.hpp>

#include <ompl/base/State.h>
#include <ompl/base/Cost.h>


namespace ob = ompl::base;

namespace rrt
{

struct NodeKey
{
  double k1 = std::numeric_limits<double>::infinity();
  double k2 = std::numeric_limits<double>::infinity();

  bool operator<(const NodeKey key) const
  {
    return k1 < key.k1 || abs(k1 - key.k1) < 0.00001 && k2  < key.k2;
  }
  bool operator>(const NodeKey key) const
  {
    return k1 > key.k1 || abs(k1 - key.k1) < 0.00001 && k2  > key.k2;
  }
};


class Motion
{
public:

    Motion *parent = nullptr;
    std::vector<Motion *> children;

    NodeKey key;

    ob::Cost  g; 
    ob::Cost  lmc;

    ob::State *state;

    std::vector<Motion *> inN0;
    std::vector<Motion *> outN0;
    
    std::vector<Motion *> inNr;
    std::vector<Motion *> outNr;

    std::vector<Motion *> inNbhs()
    {
        auto in = inN0;
        in.insert(in.end(), inNr.begin(), inNr.end());
        return in;
    }

    std::vector<Motion *> outNbhs()
    {
        auto out = outN0;
        out.insert(out.end(), outNr.begin(), outNr.end());
        return out;
    }
};

struct Trajectory
{
    Motion *source;
    Motion *target;

    double cost;
    Trajectory(){}
    Trajectory(Motion *a, Motion *b, double trajCost): source(a), target(b), cost(trajCost){}
};

typedef std::pair<Motion *, Motion *> MotionPair;
struct hash_motion_pair
{
    std::size_t operator()(const MotionPair &p) const
    {
        std::size_t seed = 0;
        seed += boost::hash<Motion *>()(p.first);
        seed ^= boost::hash<Motion *>()(p.second);
        return seed;
    }
};

struct motion_pair_equal
{
    bool operator()(const MotionPair &p1, const MotionPair &p2) const
    {
        return  p1.first == p2.first && p1.second == p2.second;
    }
};

struct motion_compare
{
    bool operator()(const Motion *v1, const Motion *v2) const
    {
        return v1->key > v2->key;
    }
};

}; // namespace

#endif