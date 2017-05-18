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
    vertex  parent;
    vertex  self;

    double   g;
    double   lmc;

    double   x;
    double   y;
};


class RRTx
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    typedef bg::model::segment<point> segment;
    typedef bg::model::box<point> box;
    typedef std::pair<point, vertex> value;
    typedef bgi::rtree< value, bgi::rstar<16> > RTree; 
    typedef boost::unordered_map<vertex, Node> Hash;
    typedef std::vector<std::pair<float, Node> > NearInfo;

    public:
                            RRTx            () {}
                            RRTx            (costmap_2d::Costmap2D *costmap);
                            ~RRTx           (){}
        void                setMaxDist      (double max_dist);
    private:
        
        void                addVertex       (Node v);
        

        std::vector<Node>   inN             (Node v);
        std::vector<Node>   outN            (Node v);
        
        std::vector<Node>   inN_s           (Node v);
        std::vector<Node>   outN_s          (Node v);
        void                addEdge_s       (Node v, Node u);

        std::vector<Node>   inN_r           (Node v);
        std::vector<Node>   outN_r          (Node v);
        void                addEdge_r       (Node v, Node u);

        std::vector<Node>   inN__           (Node v, Graph g);
        std::vector<Node>   outN__          (Node v, Graph g); 
        
        Node                parent          (Node v);
        Node                nearest         (Node v);
        NearInfo            near            (Node v, double radius);
        double              distance        (Node v, Node u);
        Node                saturate        (Node v, Node u);
        void                findParent      (Node &v, std::vector<Node> vnear);
        bool                trajectoryExist (Node v, Node u);
        void                extend          (Node v, double radius);
        
        Graph   G_s;
        Graph   G_r;
        Hash    vhash;
        RTree   rtree;

        costmap_2d::Costmap2D *costmap_;

        double maxDist;


};

};

#endif
