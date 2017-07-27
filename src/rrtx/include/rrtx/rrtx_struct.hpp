#ifndef RRTX_STRUCT_HPP
#define RRTX_STRUCT_HPP

#include <limits>
#include <stdlib.h>
#include <vector>
#include <boost/unordered_map.hpp>


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


struct Node
{
    Node                *parent = nullptr;
    std::vector<Node *> childs;

    NodeKey key;

    double  g       = std::numeric_limits<double>::infinity(); 
    double  lmc     = std::numeric_limits<double>::infinity();

    double  x;
    double  y;

    std::vector<Node *> inNz;
    std::vector<Node *> outNz;
    
    std::vector<Node *> inNr;
    std::vector<Node *> outNr; 
};

struct Trajectory
{
    Node *source;
    Node *target;

    double cost;
    Trajectory(){}
    Trajectory(Node *a, Node *b, double trajCost): source(a), target(b), cost(trajCost){}
};
typedef std::pair<Node *, Node *> NodePair;
struct hash_node_pair
{
    std::size_t operator()(const NodePair &p) const
    {
        std::size_t seed = 0;
        seed += boost::hash<Node *>()(p.first);
        seed += boost::hash<Node *>()(p.second);
        return seed;
    }
};

struct node_pair_equal
{
    bool operator()(const NodePair &p1, const NodePair &p2) const
    {
        return  p1.first == p2.first && p1.second == p2.second || 
                p1.first == p2.second && p1.second == p2.first;
    }
};

struct node_compare
{
    bool operator()(const Node *v1, const Node *v2) const
    {
        return v1->key > v2->key;
    }
};

typedef boost::unordered_map<NodePair, Trajectory, hash_node_pair, node_pair_equal> TrajectoryHash;


double angle(Node a, Node b, Node c)
{
    double x1, y1, x2, y2;

    x1 = b.x - a.x;
    y1 = b.y - a.y;

    x2 = b.x - c.x;
    y2 = b.y - c.y;

    double dot = x1*x2 + y1*y2;
    double cross = x1 * y2 - y1 * x2;

    double angle = atan2(cross, dot);

    return angle;
}



}; // namespace rrt

#endif