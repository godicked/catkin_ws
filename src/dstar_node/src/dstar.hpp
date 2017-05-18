#ifndef DSTAR_HPP
#define DSTAR_HPP

//#include <pluginlib/class_list_macros.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <costmap_2d/costmap_2d.h>
#include <stdio.h>
//#include <ros/ros.h>
//#include "global_planner.h"

namespace dstar
{

struct vertex_key
{
  float k1;
  float k2;

  bool operator<(const vertex_key key) const
  {
    return k1 < key.k1 || fabs(k1 - key.k1) < 0.00001 && k2 +0.00001 < key.k2;
  }
  bool operator>(const vertex_key key) const
  {
    return k1 > key.k1 || fabs(k1 - key.k1) < 0.00001 && k2 -0.00001 > key.k2;
  }
};

struct vertex
{
  vertex_key key;
  float g;
  float rhs;
  unsigned int x;
  unsigned int y;

  bool operator==(const vertex v) const
  {
    return x == v.x && y == v.y;
  }
};


struct vertex_compare
{
  bool operator()(const vertex *v1, const vertex *v2) const
  {
    return v1->key > v2->key;
  }
};

typedef boost::heap::fibonacci_heap<vertex *, boost::heap::compare<vertex_compare> > priority_queue_t;
typedef priority_queue_t::handle_type handle_t;
typedef boost::unordered_map<vertex *, handle_t> open_hash_t;
typedef std::list<vertex> path_t;
typedef std::vector<std::vector<vertex> > vertex_map_t;
typedef std::list<vertex *> vlist_t;
typedef costmap_2d::Costmap2D costmap_t;

class DStar
{
public:
  DStar() {}
  DStar(costmap_t costmap);
  bool init(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y);
  bool computeShortestPath();
  path_t getPath();
  costmap_t getCostmap();
  vertex_map_t getVertexMap();
  bool updateCost(unsigned int x, unsigned int y, unsigned char cost);
  void updateStart(unsigned int x, unsigned int y);
  void printInfo(vertex *v);

private:
  vlist_t getNeighbours(vertex *c);
  vertex_key calculateKey(vertex *c);
  void updateVertex(vertex *c);
  float heuristic(vertex *from, vertex *to);
  void queueInsert(vertex *c, vertex_key key);
  void queueUpdate(vertex *c, vertex_key key);
  void queueRemove(vertex *c);
  bool queueContains(vertex *c);
  vertex_key queueTopKey();
  std::pair<vertex *, float> minNeighbour(vertex *c);
  float minNeighbourRhs(vertex *c);
  float Cost(vertex *from, vertex *to);
  float getTravelCost(vertex *from, vertex *to);
  float getCost(unsigned int x, unsigned int y);
  bool buildPath();
  bool close(float a, float b);

  costmap_t costmap_;
  vertex_map_t vertex_map;
  int width;
  int height;
  priority_queue_t priority_queue;
  open_hash_t open_hash;
  path_t path;
  
  vertex *start_;
  vertex *goal_;
  float km;
};
};

#endif
