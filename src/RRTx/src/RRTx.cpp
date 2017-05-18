#include "RRTx.hpp"
#include <math.h>

using namespace std;
using namespace boost;

void removeN(list<rrt::Node *> *list, rrt::Node *u)
{
    auto it = list->begin();
    while(it != list->end())
    {
        if(*it == u)
        {
            list->erase(it);
            return;
        }

        it++;
    }
}


namespace rrt
{
    
    RRTx::RRTx(costmap_2d::Costmap2D *costmap)
    {
      costmap_ = costmap;
    }

    
    void RRTx::addVertex(Node &v)
    {
        point v_point(v.x, v.y);
        rtree.insert(make_pair(v_point, v));
    }

    list<Node *> RRTx::inN(Node v)
    {
        v.inNz.splice(v.inNz.end(), v.inNr);
        return v.inNz;
    }

    list<Node *> RRTx::outN(Node v)
    {
        v.outNz.splice(v.outNz.end(), v.outNr);
        return v.outNz;
    }


    Node *RRTx::nearest(Node v)
    {
        vector<Leaf> result;
        point p(v.x, v.y);

        rtree.query(bgi::nearest(p, 1), back_inserter(result));
        return &result[0].second; 
    }

    RRTx::NearInfoVec RRTx::near(Node v, double radius)
    {
        vector<Leaf> search;
        point p1(v.x - radius, v.y - radius), 
              p2(v.x + radius, v.y + radius);

        box b(p1, p2);

        NearInfoVec nodes;
        rtree.query(bgi::intersects(b), back_inserter(search));
        for(int i = 0; i < search.size(); i++)
        {
            Node *node  = &search[i].second;
            float dist  = distance(*node, v);

            if(dist > radius)
                continue;

            nodes.push_back(make_tuple(node, dist, false));
        }

        return nodes;
    }

    double RRTx::distance(Node v, Node u)
    {
        double dx2 = (v.x - u.x) * (v.x - u.x);
        double dy2 = (v.y - u.y) * (v.y - u.y);

        return sqrt(dx2 + dy2);
    }


    Node RRTx::saturate(Node v, Node u)
    {
        double dist = distance(v, u);

        double dx = (v.x - u.x) / dist;
        double dy = (v.y - u.y) / dist;

        v.x = u.x + dx * maxDist;
        v.y = u.y + dy * maxDist;

        return v;
    }

    void RRTx::findParent(Node &v, NearInfoVec &vnear)
    {
        for(auto near : vnear)
        {
            double dist = get<1>(near);
            Node *u     = get<0>(near);
            
            if(v.lmc > dist + u->lmc &&
               trajectoryExist(v, near))
            {
                v.parent    = u;
                v.lmc       = dist + u->lmc;
                
            }
        }
        
    }

    bool RRTx::trajectoryExist(Node v, NearInfo &near)
    {
        Node u      = *get<0>(near);
        double dist = get<1>(near);

        double dx = u.x - v.x;
        double dy = u.y - v.y;

        double resolution = costmap_->getResolution();
        for(double i = 0; i < dist; i += resolution)
        {
            double wx = v.x + dx * i;
            double wy = v.y + dy * i;

            unsigned int mx, my;
            costmap_->worldToMap(wx, wy, mx, my);
            unsigned char cost = costmap_->getCost(mx, my);
            
            if(cost > 200)
                return false;
        }

        get<2>(near) = true;
        return true;
    }

    /*  Algorithm 3: cullNeighbors(v, r)
     *
     */

    void RRTx::cullNeighbors(Node &v, double radius)
    {
        auto it = v.outNr.begin();

        while(it != v.outNr.end())
        {
            Node *u = *it;
            if(v.parent != u && radius < distance(v, *u))
            {
                it = v.outNr.erase(it);
                removeN(&u->inNr, &v);
            }
            else
            {
                it++;
            }
        }
    }

    /*  Algorithm 2: extend(v, r)
     *  
     *
     */
    void RRTx::extend(Node &v, double radius)
    {

        //Get Information about Nodes near v in the given radius
        //Each NearInfo tuple contains the target node u, the distance d(v, u),
        //and a bool saving the collision test made by trajectoryExist()
        //(called in findParent)

        NearInfoVec vnear = near(v, radius);
        findParent(v, vnear);
        if(v.parent == nullptr)
            return;

        v.parent->childs.push_back(&v);

        for(auto near : vnear)
        {
            // Trajectory from v to u already calculated by
            // findParent(v, near)
            // near = (Node *u, float dist, bool trajExist)
            Node *u         = get<0>(near);
            bool trajExist  = get<2>(near);

            if(trajExist)
            {
                v.outNz.push_back(u);
                u->inNr.push_back(&v);
            }

            // Inverse trajectory u to v.
            // Never calculate, so we call trajectoryExist(u, near)
            // near = (v, dist = same as before, trajExist = false)
            
            get<0>(near) = &v;
            get<2>(near) = false;

            if(trajectoryExist(*u, near))
            {
                u->outNr.push_back(&v);
                v.inNz.push_back(u);
            }
        }
   }

};

int main()
{
    cout << "test " << endl;
    return 1;
}





