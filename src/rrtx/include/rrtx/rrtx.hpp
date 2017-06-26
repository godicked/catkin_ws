#ifndef RRTX_HPP
#define RRTX_HPP

#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/unordered_map.hpp>
#include <boost/container/stable_vector.hpp>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


using namespace boost;

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
    typedef std::pair<point, Node *> Leaf;
    typedef bgi::rtree<Leaf, bgi::rstar<16> > RTree; 
    typedef std::tuple<Node *, float, bool> NearInfo;
    typedef std::vector<NearInfo > NearInfoVec;
    typedef boost::heap::fibonacci_heap<Node *, 
            boost::heap::compare<node_compare> > Queue;
    typedef Queue::handle_type handle_t;
    typedef boost::unordered_map<Node *, handle_t> Hash;

    public:

        typedef boost::container::stable_vector<Node> NodeContainer;
        typedef std::vector<geometry_msgs::Pose> Path;
    
        
                            RRTx            ();
                            RRTx            (costmap_2d::Costmap2D *costmap);
                            ~RRTx           (){}

        void                setCostmap      (costmap_2d::Costmap2D *costmap);
        void                setMaxDist      (double max_dist);
        void                init            (geometry_msgs::Pose start,
                                             geometry_msgs::Pose goal);
        void                init            (double sx, double sy, double gx, double gy);
        void                grow            (unsigned int iteration);
        Node                rootNode        ();
        NodeContainer       getContainer    ();
        bool                getPath         (Path &path);

        void                publish         (bool path = true, bool tree = false);

    private:
        
        void                addVertex       (Node *v);
        

        std::vector<Node *> inN             (Node v);
        std::vector<Node *> outN            (Node v);
        void                removeN         (std::vector<Node *> * vec, Node *u);
        
       
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
        void                updateRadius    ();
        Node                randomNode      ();
        bool                isObstacle      (Node v);
        void                grow            ();

        //  Priority Queue related functions
        void                queueInsert     (Node *v);
        void                queueUpdate     (Node *v);
        void                queueRemove     (Node *v);
        bool                queueContains   (Node *v);
        void                updateKey       (Node *v);
        Node               *queuePop        ();

        //  Member variables

        //  Container to hold the Node structure
        NodeContainer nodeContainer;

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


        double  epsilon = 0.05;
        double  radius;
        double  y;

        Node    *vbot_;
        Node    *goal_;

        std::vector<geometry_msgs::Point> smooth_path;

        ros::NodeHandle nh_;
        ros::Publisher  marker_pub;
        ros::Publisher  path_pub;

        std::string map_frame;



};

};

#endif
