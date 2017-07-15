//#include <boost/shared_ptr.hpp>
#include <algorithm>

#include <visualization_msgs/Marker.h>
#include <rrtx/kinematic_path_builder.hpp>

using namespace std;

namespace rrt
{


double d(Node v, Node u)
{
    double dx2 = (v.x - u.x) * (v.x - u.x);
    double dy2 = (v.y - u.y) * (v.y - u.y);

    return sqrt(dx2 + dy2);
}

vector<Node *> outN(Node v)
{
    vector<Node *> out = v.outNz;
    out.insert(out.end(), v.outNr.begin(), v.outNr.end());
    return out;
}

int depth(Waypoint *w, Waypoint *root)
{
    int counter = 0;
    for(; w != root; w = w->origin, counter++){}
    return counter;
}

KinematicPathBuilder::KinematicPathBuilder()
{

}

KinematicPathBuilder::KinematicPathBuilder(ros::NodeHandle n, double wheelbase, double max_steering)
{
    marker_pub  = n.advertise<visualization_msgs::Marker>("visited", 100);
    kmax = tan(max_steering) / wheelbase;
}

vector<Node *> KinematicPathBuilder::buildPath(Node *start, Node *goal)
{
    Waypoint *w = buildWaypoints(start, goal);
    vector<Node *> path;
    
    while(w != nullptr)
    {
        ROS_INFO("set path..");
        path.push_back(w->pos);
        w = w->origin;
    }
    ROS_INFO("reverse");
    reverse(path.begin(), path.end());
    ROS_INFO("path length %ld", path.size());
    return path;
}

Waypoint *KinematicPathBuilder::buildWaypoints(Node *start, Node *goal)
{
    queue.clear();
    root.reset();
    visited.clear();

    WaypointSharedPtr startw( new Waypoint(start));
    WaypointSharedPtr nextw( new Waypoint(start->parent, startw.get()) );

    startw->pos = start;
    startw->neighbors.push_back(nextw);
    startw->computed = true;

    nextw->pos = start->parent;
    nextw->origin = startw.get();
    nextw->cost = d(*nextw->origin->pos, *nextw->pos) + nextw->origin->cost;
    

    queue.push(nextw.get());
    Waypoint *nearest = nullptr;

    while(  !queue.empty() &&
            queue.top()->pos != goal && 
            queue.top()->cost < 3 * start->lmc)
    {
        Waypoint *w = queue.top();
        queue.pop();

        //ROS_INFO("depth %d", depth(w, startw.get()));
        if(!nearest || nearest->pos->lmc > w->pos->lmc)
        {
            nearest = w;
        }
        Waypoint *best = findBestNext(w);
        Waypoint *origin = w->origin;
        //ROS_INFO("n: %d", w->n);
        if(w->neighbors.empty())
        {
            //ROS_INFO("propagate dead end");
            propagateDeadEnd(w);
        }
        if(!best)
        {
            //ROS_INFO("push origin");
            queue.push(origin);
        }
        else
        {
            queue.push(origin);
            queue.push(best);
        }
        
        publishVisited();        
    }

    if(!queue.empty() && queue.top()->pos == goal)
    {
        nearest = queue.top();
    }

    ROS_INFO("Build Waypoints end");
    root = startw;
    return nearest;
}

void KinematicPathBuilder::propagateDeadEnd(Waypoint *w)
{
    Waypoint *origin = w->origin;
    // remove w from possible neighbors
    //ROS_INFO("size: %ld", origin->neighbors.size());
    auto n = &origin->neighbors;
    n->erase(
        remove_if(n->begin(), n->end(), [w](const WaypointSharedPtr x){ return x.get() == w; }),
        n->end()
    );
    origin->n--;
    //ROS_INFO("size: %ld", origin->neighbors.size());
    if(!origin->computed && origin->neighbors.empty())
    {
        orderNeighbors(origin);
    }
}

void KinematicPathBuilder::orderNeighbors(Waypoint *w)
{
    //ROS_INFO("order neighbors");
    vector<WaypointSharedPtr> neighbors;

    for(auto n : outN(*w->pos))
    {
        if(n == w->pos->parent) continue;
        
        WaypointSharedPtr next( new Waypoint(n, w) );
        next->cost = w->cost + d(*w->pos, *n);

        neighbors.push_back(next);
    }

    sort(neighbors.begin(), neighbors.end(), [](const WaypointSharedPtr w1, const WaypointSharedPtr &w2) { 
        return w1->cost + w1->pos->lmc < w2->cost + w2->pos->lmc; 
    });

    w->neighbors.insert(w->neighbors.end(), neighbors.begin(), neighbors.end());
    w->computed = true;
    //ROS_INFO("end order neighbors");
}


Waypoint *KinematicPathBuilder::findBestNext(Waypoint *w)
{
    //ROS_INFO("find best next");

    if(!w->computed && w->neighbors.empty())
    {
        //ROS_INFO("set parent");
        WaypointSharedPtr parent( new Waypoint(w->pos->parent, w) );
        parent->cost = w->cost + d(*w->pos, *parent->pos);
        w->neighbors.push_back(parent);   
    }

    //ROS_INFO("iterate over neighbors");
    int i;
    for(i = w->n; i < w->neighbors.size(); i++)
    {
        //ROS_INFO("for..");
        auto n = w->neighbors[i];
        
        //ROS_INFO("while..");
        if(curvature(*w->origin->pos, *w->pos, *n->pos) < kmax)
        {
            //ROS_INFO("curvature ok");
            visited.push_back(make_pair(w->pos, n->pos));
            break;
        }
        //ROS_INFO("curvature not ok");

        if(!w->computed && i+1 == w->neighbors.size())
        {
            orderNeighbors(w);
        }
    }
    //ROS_INFO("end over iterate");

    auto first = w->neighbors.begin() + w->n;
    auto last = w->neighbors.begin() + i;
    if(first < last)
    {
        w->neighbors.erase(first, last);
        //ROS_INFO("erase");
    }
    if(w->n >= w->neighbors.size())
    {
        //ROS_INFO("end best next nullptr");
        return nullptr;
    }

    Waypoint *best = w->neighbors[w->n].get();
    w->n++;
    //ROS_INFO("end best next");
    return best;
}

// WaypointSharedPtr KinematicPathBuilder::findBestAlternative(Waypoint *w)
// {
//     WaypointSharedPtr best;
    
//     for(auto n : outN(*w->pos))
//     {
//         if(n == w->pos->parent) continue;

//         if(curvature(*w->origin->pos, *w->pos, *n) < kmax)
//         {
//             WaypointSharedPtr next( new Waypoint );
//             next->pos = n;
//             next->origin = w;
//             next->cost = w->cost + d(*w->pos, *n);
//             w->neighbors.push_back(next);
//             visited.push_back(make_pair(w->pos, next->pos));
//             queue.push(next);

//             if(!best || (best->cost + best->pos->lmc) > (next->cost + next->pos->lmc))
//             {
//                 best = next;
//             }
//         }
//     }

//     w->computed = true;
//     return best;
// }

double KinematicPathBuilder::angle(Node a, Node b, Node c)
{
    double x1, y1, x2, y2;

    x1 = b.x - a.x;
    y1 = b.y - a.y;

    x2 = b.x - c.x;
    y2 = b.y - c.y;

    double dot = x1*x2 + y1*y2;
    double cross = x1 * y2 - y1 * x2;

    double angle = atan2(cross, dot);

    return angle;
}

double KinematicPathBuilder::curvature(Node a, Node b, Node c)
{
    double alpha = angle(a, b, c);
    double lmin = min(d(a, b), d(b, c));

    if(fabs(alpha) < 0.001) return std::numeric_limits<double>::max();

    double term1 = sin(alpha);
    double term2 = (1 / 8.0) * (1 - cos(alpha));
    double k = term1 / (6 * lmin * pow(term2, 3/2.0));

    return fabs(k);
}

void KinematicPathBuilder::publishVisited()
{
    visualization_msgs::Marker edges;
    edges.header.frame_id  = "map";
    edges.header.stamp     = ros::Time::now();
    edges.ns               = "visited";
    edges.id               = 6; 
    edges.action           = visualization_msgs::Marker::ADD;
    edges.type             = visualization_msgs::Marker::LINE_LIST;

    edges.scale.x          = 0.02;
    edges.scale.y          = 0.02;

    edges.color.a          = 1;
    edges.color.r          = 1;

    for(auto e : visited)
    {
        geometry_msgs::Point p1, p2;
        p1.x = e.first->x;
        p1.y = e.first->y;
        p2.x = e.second->x;
        p2.y = e.second->y;

        edges.points.push_back(p1);
        edges.points.push_back(p2);
    }

    marker_pub.publish(edges);
}


}; // namespace rrt
