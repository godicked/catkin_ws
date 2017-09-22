#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>


#include <rrtx/rrtx_struct.hpp>

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/tools/config/SelfConfig.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using namespace boost;
using namespace rrt;


typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::segment<point> segment;
typedef bg::model::box<point> box;

typedef std::pair<box, int> RTreeBox;
typedef bgi::rtree<RTreeBox, bgi::rstar<16> > NodeRTree;


void insert_segment(NodeRTree &tree, point a, point b, int id)
{
    box bb(a, b);
    RTreeBox leaf = std::make_pair(bb, id);
    tree.insert(leaf);
}

std::vector<RTreeBox> find_intersection(NodeRTree &tree, box s)
{
    std::cout << "start search" << std::endl;
    std::vector<RTreeBox> result;
    std::vector<int> final_result;
    tree.query(bgi::intersects(s), back_inserter(result));
    for(auto res : result)
    {
        if(bg::intersects(segment(res.first.min_corner(), res.first.max_corner()), segment(s.min_corner(), s.max_corner())))
        {
            std::cout << res.second << std::endl;
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    // NodeRTree rtree;
    // insert_segment(rtree, point(1,1), point(2,2), 0);
    // find_intersection(rtree, box(point(2,1), point(1,2)));
    // find_intersection(rtree, box(point(1,1), point(2,2)));
    // find_intersection(rtree, box(point(1,2), point(2,2)));
    // find_intersection(rtree, box(point(1.01,1), point(2.01,2)));

    //std::shared_ptr<ompl::NearestNeighbors<Node *>> nn;
    //nn.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Node *>());



}