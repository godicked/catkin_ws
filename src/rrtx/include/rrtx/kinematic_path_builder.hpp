#ifndef KINEMATIC_PATH_BUILDER_HPP
#define KINEMATIC_PATH_BUILDER_HPP

#include <rrtx/rrtx_struct.hpp>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

namespace rrt
{

typedef struct Waypoint Waypoint;
typedef boost::shared_ptr<Waypoint> WaypointSharedPtr;

struct Waypoint
{
    Node *pos = nullptr;
    Waypoint *origin = nullptr;
    std::vector<WaypointSharedPtr> neighbors;
    double cost = 0;
    bool computed = false;
    int n = 0;
    Waypoint(Node *n) : pos(n)
    {}
    Waypoint(Node *n, Waypoint *origin) : pos(n), origin(origin)
    {}
};


double waypoint_key(const Waypoint *w) 
{
    return w->cost + w->pos->lmc;
}

struct waypoint_compare
{
    const bool operator()(const Waypoint *w1, const Waypoint *w2) const
    {
        return waypoint_key(w1) > waypoint_key(w2);
    }
};


class KinematicPathBuilder
{
    typedef boost::heap::fibonacci_heap<Waypoint *, boost::heap::compare<waypoint_compare> > Queue;
    
    public:
        KinematicPathBuilder();
        KinematicPathBuilder(ros::NodeHandle n, double wheelbase, double max_steering);
        std::vector<Node *> buildPath(Node *start, Node *goal);
        void publishVisited();

    private:
        Waypoint *buildWaypoints(Node *start, Node *goal);
        Waypoint *findBestNext(Waypoint *w);
        void propagateDeadEnd(Waypoint *w);
        double angle(Node a, Node b, Node c);
        double curvature(Node a, Node b, Node c);
        void orderNeighbors(Waypoint *w);


        Queue queue;
        double kmax;
        WaypointSharedPtr root;
        std::vector<std::pair<Node *, Node *> > visited;
        ros::Publisher marker_pub;



};
}
; // namespace rrt

#endif