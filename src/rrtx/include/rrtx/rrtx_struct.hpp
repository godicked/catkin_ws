#ifndef RRTX_STRUCT_HPP
#define RRTX_STRUCT_HPP

#include <limits>
#include <stdlib.h>
#include <vector>

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

}; // namespace rrt

#endif