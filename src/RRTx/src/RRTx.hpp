#ifndef RRTX_HPP
#define RRTX_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include <costmap_2d/costmap_2d.h>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


using namespace boost;

namespace rrt
{


struct NodeKey
{
  double k1;
  double k2;

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
    Node               *parent = nullptr;
    std::list<Node *>   childs;

    NodeKey key;

    double  g       = std::numeric_limits<double>::infinity();
    double  lmc     = std::numeric_limits<double>::infinity();

    double  x;
    double  y;

    std::list<Node *> inNz;
    std::list<Node *> outNz;
    
    std::list<Node *> inNr;
    std::list<Node *> outNr; 
};

struct node_compare
{
    bool operator()(const Node *v1, const Node *v2) const
    {
        return v1->key > v2->key;
    }
};


class RRTx
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    //typedef bg::model::segment<point> segment;
    typedef bg::model::box<point> box;
    typedef std::pair<point, Node> Leaf;
    typedef bgi::rtree<Leaf, bgi::rstar<16> > RTree; 
    typedef std::tuple<Node *, float, bool> NearInfo;
    typedef std::vector<NearInfo > NearInfoVec;
    typedef boost::heap::fibonacci_heap<Node *, 
            boost::heap::compare<node_compare> > Queue;
    typedef Queue::handle_type handle_t;
    typedef boost::unordered_map<Node *, handle_t> Hash; 

    public:
                            RRTx            (){}
                            RRTx            (costmap_2d::Costmap2D *costmap);
                            ~RRTx           (){}
        void                setMaxDist      (double max_dist);
    private:
        
        void                addVertex       (Node *v);
        

        std::list<Node *>   inN             (Node v);
        std::list<Node *>   outN            (Node v);
        
       
        Node               *nearest         (Node v);
        NearInfoVec         near            (Node v);
        double              distance        (Node v, Node u);
        Node                saturate        (Node v, Node u);
        void                findParent      (Node *v, NearInfoVec &vnear);
        bool                trajectoryExist (Node v, NearInfo &near);
        void                extend          (Node *v);
        void                cullNeighbors   (Node *v);
        void                verrifyQueue    (Node *v);
        void                rewireNeighbors (Node *v);
        void                reduceInconsist ();
        void                updateLMC       (Node *v);
        double              ballRadius      ();

        //  Priority Queue related functions
        void                queueInsert     (Node *v);
        void                queueUpdate     (Node *v);
        void                queueRemove     (Node *v);
        bool                queueContains   (Node *v);
        void                updateKey       (Node *v);

        //  Member variables

        //  R* Tree for efficient spacial k nearest neighbors (KNN) search
        //  and helps for the radius search
        RTree   rtree;

        //  The priority queue needed by the RRTx algorithm
        //  And a hash table to save the handle of the inserted object
        //  Allows contains/updade/remove operations
        Queue   queue;
        Hash    hash;

        //  A costmap for the collision tests
        costmap_2d::Costmap2D *costmap_;

        //  max distance between 2 a new point and its nearest neighbor
        double maxDist;


        double  epsilon;
        double  radius;
        double  y;

        Node    *vbot;


};

};

#endif