#ifndef KINEMATIC_PATH_BUILDER_HPP
#define KINEMATIC_PATH_BUILDER_HPP

#include <rrtx/rrtx_struct.hpp>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/shared_ptr.hpp>

namespace rrt
{

typedef struct Waypoint Waypoint;
typedef boost::shared_ptr<Waypoint> WaypointSharedPtr;

struct Waypoint
{
    Node *pos = nullptr;
    boost::shared_ptr<Waypoint> origin;
    std::vector<WaypointSharedPtr> neighbors;
    double cost = 0;
    bool computed = false;
};


double waypoint_key(const WaypointSharedPtr w) 
{
    return w->cost + w->pos->lmc;
}

struct waypoint_compare
{
    const bool operator()(const WaypointSharedPtr w1, const WaypointSharedPtr w2) const
    {
        return waypoint_key(w1) > waypoint_key(w2);
    }
};


class KinematicPathBuilder
{
    typedef boost::heap::fibonacci_heap<WaypointSharedPtr, boost::heap::compare<waypoint_compare> > Queue;
    
    public:
        KinematicPathBuilder(double wheelbase, double max_steering);
        KinematicPathBuilder(double max_curvature);
        std::vector<Node *> buildPath(Node *start, Node *goal);

    private:
        WaypointSharedPtr buildWaypoints(Node *start, Node *goal);
        WaypointSharedPtr findBestNext(WaypointSharedPtr w);
        WaypointSharedPtr findBestAlternative(WaypointSharedPtr w);
        void propagateDeadEnd(WaypointSharedPtr w);
        double angle(Node a, Node b, Node c);
        double curvature(Node a, Node b, Node c);

        Queue queue;
        double kmax;



};
}
; // namespace rrt

#endif