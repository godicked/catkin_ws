#include "RRTx.hpp"
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace boost;

const double infinity = numeric_limits<double>::infinity();


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

    
    void RRTx::addVertex(Node *v)
    {
        point v_point(v->x, v->y);
        rtree.insert(make_pair(v_point, *v));
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

    RRTx::NearInfoVec RRTx::near(Node v)
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

    void RRTx::findParent(Node *v, NearInfoVec &vnear)
    {
        for(auto near : vnear)
        {
            double dist = get<1>(near);
            Node *u     = get<0>(near);
            
            if(v->lmc > dist + u->lmc &&
               trajectoryExist(*v, near))
            {
                v->parent    = u;
                v->lmc       = dist + u->lmc;
                
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

    bool RRTx::isObstacle(Node v)
    {
        unsigned int mx, my;
        costmap_->worldToMap(v.x, v.y, mx, my);
        unsigned char cost = costmap_->getCost(mx, my);

        if(cost > 200)
            return true;
    }

    /*  Algorithm 3: cullNeighbors(v, r)
     *  radius is a member variable
     */

    void RRTx::cullNeighbors(Node *v)
    {
        auto it = v->outNr.begin();

        while(it != v->outNr.end())
        {
            Node *u = *it;
            if(v->parent != u && radius < distance(*v, *u))
            {
                it = v->outNr.erase(it);
                removeN(&u->inNr, v);
            }
            else
            {
                it++;
            }
        }
    }

    /*  Algorithm 2: extend(v, r)
     *  radius is a member variable
     *
     */
    void RRTx::extend(Node *v)
    {

        //Get Information about Nodes near v in the given radius
        //Each NearInfo tuple contains the target node u, the distance d(v, u),
        //and a bool saving the collision test made by trajectoryExist()
        //(called in findParent)

        NearInfoVec vnear = near(*v);
        findParent(v, vnear);
        if(v->parent == nullptr)
            return;

        v->parent->childs.push_back(v);

        for(auto near : vnear)
        {
            // Trajectory from v to u already calculated by
            // findParent(v, near)
            // near = (Node *u, float dist, bool trajExist)
            Node *u         = get<0>(near);
            bool trajExist  = get<2>(near);

            if(trajExist)
            {
                v->outNz.push_back(u);
                u->inNr.push_back(v);
            }

            // Inverse trajectory u to v.
            // Never calculate, so we call trajectoryExist(u, near)
            // near = (v, dist = same as before, trajExist = false)
            
            get<0>(near) = v;
            get<2>(near) = false;

            if(trajectoryExist(*u, near))
            {
                u->outNr.push_back(v);
                v->inNz.push_back(u);
            }
        }
    }

    void RRTx::verrifyQueue(Node *v)
    {
        if(queueContains(v))
            queueUpdate(v);
        else
            queueInsert(v);
    }

    void RRTx::rewireNeighbors(Node *v)
    {
        if( v->g - v->lmc > epsilon)
        {
            cullNeighbors(v);
            for(Node *u : inN(*v))
            {
                if(u == v->parent)
                    continue;

                double dist = distance(*u, *v) + v->lmc;
                if(u->lmc > dist)
                {
                    u->lmc      = dist;
                    u->parent   = v;

                    if(u->g - u->lmc > epsilon)
                        verrifyQueue(u);
                }
            }
        }
    }
        

    void RRTx::reduceInconsist()
    {
        while(  !queue.empty() &&
                (queue.top()->key < vbot->key ||
                abs(vbot->lmc - vbot->g) > 0.00001) ||
                vbot->g == infinity ||
                queueContains(vbot))
        {

            //Take first Node from queue and remove it
            Node *v = queue.top();
            queue.pop();

            if(v->g - v->lmc > epsilon)
            {
                updateLMC(v);
                rewireNeighbors(v);
            }

            v->g = v->lmc;
        }
    }

    void RRTx::updateLMC(Node *v)
    {
        cullNeighbors(v);
        Node *p = v->parent;

        for(Node *u : outN(*v))
        {
            if(u->parent == v)
                continue;

            double dist = distance(*v, *u) + u->lmc;
            if(v->lmc > dist)
            {
                p       = u;
                v->lmc  = dist;
            }
        }

        v->parent = p;
    }

	void RRTx::queueInsert(Node *v)
	 {
	    updateKey(v); 
	    hash[v] = queue.push(v);
	 }
	 
	 bool RRTx::queueContains(Node *v)
	 {
	    return hash.find(v) != hash.end();
	 }
	 
	 void RRTx::queueUpdate(Node *v)
	 {
	    updateKey(v);
	    queue.update(hash[v]);
	 }
	 
	 void RRTx::queueRemove(Node *v)
	 {
	    queue.erase(hash[v]);
	    hash.erase(v);
	 }

     void RRTx::updateKey(Node *v)
     {
         v->key.k1 = min(v->g, v->lmc);
         v->key.k2 = v->g;
     }

     void RRTx::updateRadius()
     {
         int    n       = rtree.size() + 1;
         double term1   = (y / M_PI) * (log(n) / n);
         radius  = min( pow(term1, 0.5), maxDist);
     }


     Node RRTx::randomNode()
     {
         Node node;
         
         node.x = rand() % costmap_->getSizeInCellsX();
         node.y = rand() % costmap_->getSizeInCellsY();
         
         return node;
     }

     Node RRTx::rootNode()
     {
         cout << "root node" << endl;
         return *goal_;
     }

     void RRTx::grow()
     {
         updateRadius();

         Node v         = randomNode();
         Node *vnearest = nearest(v);

         if(distance(v, *vnearest) > maxDist)
             v = saturate(v, *vnearest);

         if(!isObstacle(v))
             extend(&v);

         if(v.parent != nullptr)
         {
             rewireNeighbors(&v);
             reduceInconsist();
         }
     }

     void RRTx::grow(unsigned int iteration)
     {
         for(int i = 0; i < iteration; i++)
             grow();
     }

     void RRTx::init(int sx, int sy, int gx, int gy)
     {
         rtree.clear();
         
         Node start;
         start.x = sx;
         start.y = sy;

         Node goal;
         goal.x     = gx;
         goal.y     = gy;
         goal.g     = 0;
         goal.lmc   = 0;

         vbot   = &start;
         goal_  = &goal;

         addVertex(goal_);

     }

     void RRTx::setMaxDist(double dist)
     {
         maxDist = dist;
     }

};

void recursive_fill(visualization_msgs::Marker &points, visualization_msgs::Marker &lines, rrt::Node node)
{
    geometry_msgs::Point p;
    p.x = node.x;
    p.y = node.y;
    p.z = 1;

    points.points.push_back(p);


    for(auto child : node.childs)
    {
        geometry_msgs::Point pChild;
        pChild.x = child->x;
        pChild.y = child->y;
        pChild.z = 1;

        lines.points.push_back(p);
        lines.points.push_back(pChild);

        recursive_fill(points, lines, *child);
    }
}

void fillmarker(visualization_msgs::Marker &points, visualization_msgs::Marker &lines, rrt::Node rootNode)
{
    points.header.frame_id = lines.header.frame_id = "/map";
    points.header.stamp = lines.header.stamp = ros::Time::now();
    points.ns = lines.ns = "rrtx";

    points.pose.orientation.w = lines.pose.orientation.w = 1.0;

    points.id = 0;
    lines.id = 0;

    points.type = visualization_msgs::Marker::POINTS;
    lines.type = visualization_msgs::Marker::LINE_LIST;

    points.scale.x = 0.1;
    points.scale.y = 0.1;

    lines.scale.x = 0.1;

    points.color.a = 1;
    points.color.b = 1;

    lines.color.a = 1;
    lines.color.g = 1;

    recursive_fill(points, lines, rootNode);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RRTx");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("rrtx_viz", 10);

    costmap_2d::Costmap2D costmap(100, 100, 0.1, 0, 0);



    rrt::RRTx rrt(&costmap);
    rrt.setMaxDist(0.3);
    rrt.init(2,2, 5,7); 
    rrt.grow(10);

    cout << "Get root node ... " << endl;
    rrt::Node rootNode = rrt.rootNode();
    cout << "test" << endl;
    cout << "root Node: " << rootNode.x << " " << rootNode.y << endl;



    visualization_msgs::Marker points, lines;
    fillmarker(points, lines, rootNode);

    ros::Rate rate(200);

    while(ros::ok())
    {
        marker_pub.publish(points);
        marker_pub.publish(lines);

        rate.sleep();
    }

    

    return 0;
}




