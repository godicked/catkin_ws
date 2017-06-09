#include "dstar.hpp"


namespace dstar
{
float infinity = std::numeric_limits<float>::infinity();

std::size_t hash_value(vertex const &v)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, v.x);
  boost::hash_combine(seed, v.y);
  return seed;
}

DStar::DStar(costmap_2d::Costmap2D *costmap)
{
    costmap_ = costmap;
    width = costmap_->getSizeInCellsX();
    height = costmap_->getSizeInCellsY();

    std::vector<vertex> column(height);
    vertex_map.resize(width, column);
}




costmap_2d::Costmap2D *DStar::getCostmap()
{
    return costmap_;
}

std::vector<std::vector<vertex> > DStar::getVertexMap()
{
  return vertex_map;
}

void DStar::printInfo(vertex *v)
{
  printf("\nx: %d,y: %d, rhs: %.2f, g: %.2f, k1: %.2f, k2: %.2f, Cost: %.2f, h(start_, v): %.2f\n------\n", v->x, v->y, v->rhs, v->g, v->key.k1, v->key.k2, getCost(v->x, v->y), heuristic(start_, v));
  vlist_t neighbours = getNeighbours(v);
  vlist_t::iterator it;
  for (it = neighbours.begin(); it != neighbours.end(); it++)
  {
    vertex *s = *it;
    printf("x: %d,y: %d, rhs: %.2f, g: %.2f, Cost: %.2f, g + Cost: %.3f\n", s->x, s->y, s->rhs, s->g, Cost(v,s), s->g + Cost(v, s));
  }
  //std::pair<vertex *, float> min_n = minNeighbour(v);
  // printf("min: %d,%d, travelCost: %.2f\n", min_n.first->x, min_n.first->y, min_n.second);

}

bool DStar::init(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y)
{

  for (unsigned int x = 0; x < width; x++)
  {
    for (unsigned int y = 0; y < height; y++)
    {
      vertex *u = &vertex_map[x][y];
      u->g = infinity;
      u->rhs = infinity;
      u->x = x;
      u->y = y;
    }
  }

  priority_queue.clear();
  open_hash.clear();
  km = 0;
  start_ = &vertex_map[start_x][start_y];

  goal_ = &vertex_map[goal_x][goal_y];
  goal_->rhs = 0;

  queueInsert(goal_, calculateKey(goal_));

  //ROS_INFO("Initialized Start: x: %d, y: %d", start_->x, start_->y);
  //ROS_INFO("Initialized Goal: x: %d, y: %d", goal_->x, goal_->y);

  return true;
}

void DStar::updateCost(unsigned int x, unsigned int y, unsigned char cost)
{
  if(costmap_->getCost(x, y) == cost)
    return;

  printf("update cost %d,%d ... \n", x, y);
  float old_cost = getCost(x, y);
  costmap_->setCost(x, y, cost);
  float new_cost = getCost(x, y);
  //printf("old_cost: %.2f new_cost: %.2f\n", old_cost, new_cost);

  

  vertex *v = &vertex_map[x][y];
  vlist_t neighbours = getNeighbours(v);
  vlist_t::iterator it;
  for(it = neighbours.begin(); it != neighbours.end(); it++)
  {
    vertex *n = *it;
    //printf("n: %d,%d  \n", n->x, n->y);
    if(old_cost > new_cost)
    {
      if(n != goal_)
      {
        n->rhs = std::min(n->rhs, Cost(n, v) + v->g);
      }
    }
    else
    {
      float old_rhs = getTravelCost(n, v) * old_cost + v->g;
      if (fabs(n->rhs - old_rhs) < 0.00001)
      {
        if(n != goal_)
        {
          n->rhs = minNeighbour(n).second;
          //ROS_INFO("new n rhs: %.1f", n->rhs);     
        }
      }
    }
    updateVertex(n);
  }
  //printf("update cost end\n");
}

void DStar::queueInsert(vertex *v, vertex_key key)
{
  //ROS_INFO("queueInsert vertex x: %d, y: %d, g: %1.f, rhs: %1.f", v->x, v->y, v->g, v->rhs);
  v->key = key;

  open_hash[v] = priority_queue.push(v);
}

bool DStar::queueContains(vertex *v)
{
  //ROS_INFO("queueContains(vertex *v)");
  return open_hash.find(v) != open_hash.end();
}

void DStar::queueUpdate(vertex *v, vertex_key key)
{
  //ROS_INFO("queueUpdate(vertex *v)");
  v->key = key;
  priority_queue.update(open_hash[v]);
}

void DStar::queueRemove(vertex *v)
{
  //ROS_INFO("queueRemove(vertex *v)");
  priority_queue.erase(open_hash[v]);
  open_hash.erase(v);
}

float DStar::heuristic(vertex *from, vertex *to)
{
  //ROS_INFO("heuristic(vertex *v)");
  int dx = to->x - from->x;
  int dy = to->y - from->y;

  return sqrt(dx*dx + dy*dy);
  
}

vertex_key DStar::calculateKey(vertex* v)
{
  vertex_key key;
  key.k1 = std::min(v->g, v->rhs) + heuristic(start_, v) + km;
  key.k2 = std::min(v->g, v->rhs);

  return key;
}

void DStar::updateVertex(vertex *v)
{
  //printf("update %d,%d\n", v->x, v->y);
  bool contains = queueContains(v);
  if (v->g != v->rhs && contains)
  {
    queueUpdate(v, calculateKey(v));
  }
  else if (v->g != v->rhs && !contains)
  {
    queueInsert(v, calculateKey(v));
  }
  else if (v->g == v->rhs && contains)
  {
    queueRemove(v);
  }
  //printf("end update\n");
}

vertex_key DStar::queueTopKey()
{
  //ROS_INFO("queueTopKey()");
  return priority_queue.top()->key;
}

std::list<vertex *> DStar::getNeighbours(vertex *v)
{
  //ROS_INFO("getNeighbours(vertex *v)");
  std::list<vertex *> neighbours;
  unsigned int x = v->x;
  unsigned int y = v->y;

  if (x > 0)
  {
    neighbours.push_back(&vertex_map[x - 1][y]);
  }
  if (x > 0 && y > 0)
  {
    neighbours.push_back(&vertex_map[x - 1][y - 1]);
  }
  if (x > 0 && y < height)
  {
    neighbours.push_back(&vertex_map[x - 1][y + 1]);
  }
  if (x < width)
  {
    neighbours.push_back(&vertex_map[x + 1][y]);
  }
  if (x < width && y > 0)
  {
    neighbours.push_back(&vertex_map[x + 1][y - 1]);
  }
  if (x < width && y < height)
  {
    neighbours.push_back(&vertex_map[x + 1][y + 1]);
  }
  if (y > 0)
  {
    neighbours.push_back(&vertex_map[x][y - 1]);
  }
  if (y < height)
  {
    neighbours.push_back(&vertex_map[x][y + 1]);
  }

  return neighbours;
}

float DStar::Cost(vertex *from, vertex *to)
{
  float cost = getCost(to->x, to->y);
  return  cost * getTravelCost(from, to);
}

float DStar::getCost(unsigned int x, unsigned int y)
{
  float c = costmap_->getCost(x, y);
  if(c > 250)
    c = infinity;
  return c + 1;
}

float DStar::getTravelCost(vertex *from, vertex *to)
{
    float dist = abs(from->x - to->x) + abs(from->y - to->y);
    if(dist == 2)
    dist = M_SQRT2;
    return dist;
}

bool DStar::computeShortestPath()
{
  int counter = 0;
  printf("Start computeShortestPath\n");
  //ROS_INFO("Start computeShortesPath()");
  while (!priority_queue.empty() &&
        (queueTopKey() < calculateKey(start_) 
        || start_->rhs != start_->g))
  {

    vertex *u = priority_queue.top();
    vertex_key k_old = u->key;
    vertex_key k_new = calculateKey(u);

    //printf("%d : %d,%d\n", counter, u->x, u->y);
    //printInfo(u);
    
    counter++;
    
    if (k_old < k_new)
    {
      //printf("k_old < k_new\n");
      queueUpdate(u, k_new);
    }


    else if (u->g -0.00001 > u->rhs)
    {
      //printf("g(u): %.2f > rhs(u): %.2f\n", u->g, u->rhs);
      //ROS_INFO("v->g > v-rhs");
      u->g = u->rhs;
      queueRemove(u);

      vlist_t neighbours = getNeighbours(u);
      vlist_t::iterator it;
      for (it = neighbours.begin(); it != neighbours.end(); it++)
      {
        vertex *n = *it;

        if (n != goal_)
        {
          //printf("%d,%d  rhs(n): %.2f ", n->x, n->y, n->rhs);
          n->rhs = std::min(n->rhs, Cost(n, u) + u->g);
          //ROS_INFO("new n rhs: %.1f", n->rhs);
          //printf("to: %.2f\n", n->rhs);
        }
        updateVertex(n);
      }
    }

    else
    {
      //printf("else \n");
      float g_old = u->g;
      u->g = infinity;

      vlist_t toUpdate = getNeighbours(u);
      toUpdate.push_back(u);
      vlist_t::iterator it;
      for (it = toUpdate.begin(); it != toUpdate.end(); it++)
      {
        vertex *s = *it;

        if (close(s->rhs, Cost(s, u) + g_old))
        {
          if (s != goal_)
          {
            s->rhs = minNeighbour(s).second;
          }
        }
        updateVertex(s);
      }
    }
    //ROS_INFO("vertex x: %d, y: %d, g: %.1f, rhs: %.1f", v->x, v->y, v->g, v->rhs);
  } // while

  //printf("End computeShortestPath\n");
  printf("%d node processed\n", counter);
  return buildPath();
}

bool DStar::buildPath() 
{
  //ROS_INFO("BuildPath()");
  printf("buildPath start ... \n");
  vertex *v = start_;
  vertex *s1 = NULL;
  vertex *s2 = NULL;

  path.clear();
  path.push_back(*v);

  while(v != goal_)
  {
    v = minNeighbour(v).first;
    path.push_back(*v);

    if(s2 != NULL && *s2 == *v)
    {
      return false;
    }

    s2 = s1;
    s1 = v;
  }

  printf("end\n");
  return true;
}

path_t DStar::getPath()
{
  return path;
}

bool DStar::close(float a, float b)
{
  if(a == infinity && b == infinity)
    return true;
  return fabs(a-b) < 0.0001;
}

std::pair<vertex *, float> DStar::minNeighbour(vertex *v)
{
  //printf("minNeighbour start\n");
  vertex *min_vertex = NULL;
  float min_cost = infinity;
  float min_heuristic = infinity;

  vlist_t neighbours = getNeighbours(v);
  vlist_t::iterator it;
  for (it = neighbours.begin(); it != neighbours.end(); it++)
  {
    vertex *n = *it;

    float cost = Cost(v, n) + n->g;
    float h = heuristic(start_, n) + heuristic(n, goal_);

    if (cost < min_cost ||
        (close(cost, min_cost) &&
          h < min_heuristic))
    {
      min_vertex = n;
      min_cost = cost;
      min_heuristic = h;
    }

  }
  //printf("minNeighbour end %.f\n", min_cost);
  return std::pair<vertex *, float>(min_vertex, min_cost);
}

float DStar::minNeighbourRhs(vertex *v)
{
  float min_cost = infinity;

  vlist_t neighbours = getNeighbours(v);
  vlist_t::iterator it;
  for (it = neighbours.begin(); it != neighbours.end(); it++)
  {
    vertex *n = *it;

    float cost = Cost(v, n) + n->g;
    if (cost < min_cost)
    {
      min_cost = cost;
    }
  }

  return min_cost;
}

};