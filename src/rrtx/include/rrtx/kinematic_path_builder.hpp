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
    Node *node = nullptr;
    WaypointSharedPtr origin;
    double cost = 0;
    Waypoint(Node *n) : node(n)
    {}
    Waypoint(Node *n, WaypointSharedPtr origin) : node(n), origin(origin)
    {}
};


double waypoint_key(const WaypointSharedPtr w) 
{
    return w->cost + w->node->lmc;
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
        KinematicPathBuilder();
        KinematicPathBuilder(ros::NodeHandle n, double wheelbase, double max_steering);
        std::vector<Node *> buildPath(Node *start, Node *goal);
        void publishVisited();

    private:
        WaypointSharedPtr buildWaypoints(Node *start, Node *goal);
        // Waypoint *findBestNext(Waypoint *w);
        void insertNeighbors(WaypointSharedPtr w);
        // void propagateDeadEnd(Waypoint *w);
        double angle(Node a, Node b, Node c);
        double curvature(Node a, Node b, Node c);
        double curvature(WaypointSharedPtr w);
        // void orderNeighbors(Waypoint *w);


        Queue queue;
        double kmax;
        WaypointSharedPtr root;
        std::vector<std::pair<Node *, Node *> > visited;
        ros::Publisher marker_pub;



};
}
; // namespace rrt

#endif