#include <rrtx/kinematic_path_builder.hpp>
#include <boost/shared_ptr.hpp>
#include <algorithm>


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

KinematicPathBuilder::KinematicPathBuilder(double wheelbase, double max_steering)
{
    kmax = tan(max_steering) / wheelbase;
}

KinematicPathBuilder::KinematicPathBuilder(double max_curvature)
{
    kmax = max_curvature;
}

vector<Node *> KinematicPathBuilder::buildPath(Node *start, Node *goal)
{
    WaypointSharedPtr w = buildWaypoints(start, goal);
    vector<Node *> path;
    
    while(w != nullptr)
    {
        path.push_back(w->pos);
        w = w->origin;
    }

    reverse(path.begin(), path.end());
    return path;
}

WaypointSharedPtr KinematicPathBuilder::buildWaypoints(Node *start, Node *goal)
{
    WaypointSharedPtr startw( new Waypoint());
    WaypointSharedPtr nextw( new Waypoint());

    startw->pos = start;

    nextw->pos = start->parent;
    nextw->origin = startw;
    nextw->cost = d(*nextw->origin->pos, *nextw->pos) + nextw->origin->cost;

    queue.push(nextw);
    WaypointSharedPtr lastBest;

    while(  !queue.empty() &&
            queue.top()->pos != goal && 
            queue.top()->cost < 3 * start->lmc)
    {
        WaypointSharedPtr w = queue.top();
        lastBest = w;
        queue.pop();

        WaypointSharedPtr best = findBestNext(w);
        if(!best)
        {
            propagateDeadEnd(w);
        }
        
    }
    if(!queue.empty() && queue.top()->pos == goal)
    {
        lastBest = queue.top();
    }

    return lastBest;
}

void KinematicPathBuilder::propagateDeadEnd(WaypointSharedPtr w)
{
    WaypointSharedPtr origin = w->origin;
    w->origin.reset();

    if(!origin->computed)
    {
        findBestAlternative(origin);
    }

    if(w->pos != origin->pos->parent)
    {
        auto it = find(origin->neighbors.begin(), origin->neighbors.end(), w);
        origin->neighbors.erase(it); 
    }

    if(w->neighbors.size() == 0)
    {
        propagateDeadEnd(origin);
    }

}

WaypointSharedPtr KinematicPathBuilder::findBestNext(WaypointSharedPtr w)
{
    if(curvature(*w->origin->pos, *w->pos, *w->pos->parent) < kmax)
    {
        WaypointSharedPtr best( new Waypoint );
        best->pos = w->pos->parent;
        best->origin = w;
        best->cost = w->cost + d(*w->pos, *w->pos->parent);
        queue.push(best);
        return best;
    }
    else
    {
        return findBestAlternative(w);
    }
}

WaypointSharedPtr KinematicPathBuilder::findBestAlternative(WaypointSharedPtr w)
{
    WaypointSharedPtr best;
    
    for(auto n : outN(*w->pos))
    {
        if(n == w->pos->parent) continue;

        if(curvature(*w->origin->pos, *w->pos, *n) < kmax)
        {
            WaypointSharedPtr next( new Waypoint );
            next->pos = n;
            next->origin = w;
            next->cost = w->cost + d(*w->pos, *n);
            w->neighbors.push_back(next);
            queue.push(next);

            if(!best || (best->cost + best->pos->lmc) > (next->cost + next->pos->lmc))
            {
                best = next;
            }
        }
    }

    w->computed = true;
    return best;
}

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



}; // namespace rrt
