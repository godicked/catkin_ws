#ifndef RRTX_HPP
#define RRTX_HPP

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


typedef boost::adjacency_list<vecS, listS, bidirectionalS> Graph;
typedef Graph::vertex_descriptor vertex;

struct Node
{
    Node               *parent = nullptr;
    std::list<Node *>   childs;

    double  g;
    double  lmc;

    double  x;
    double  y;

    std::list<Node *> inNz;
    std::list<Node *> outNz;
    
    std::list<Node *> inNr;
    std::list<Node *> outNr; 
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

    public:
                            RRTx            () {}
                            RRTx            (costmap_2d::Costmap2D *costmap);
                            ~RRTx           (){}
        void                setMaxDist      (double max_dist);
    private:
        
        void                addVertex       (Node &v);
        

        std::list<Node *>   inN             (Node v);
        std::list<Node *>   outN            (Node v);
        
       
        Node               *nearest         (Node v);
        NearInfoVec         near            (Node v, double radius);
        double              distance        (Node v, Node u);
        Node                saturate        (Node v, Node u);
        void                findParent      (Node &v, NearInfoVec &vnear);
        bool                trajectoryExist (Node v, NearInfo &near);
        void                extend          (Node &v, double radius);
        void                cullNeighbors   (Node &v, double radius);
        

        RTree   rtree;
        costmap_2d::Costmap2D *costmap_;

        double maxDist;


};

};

#endif
