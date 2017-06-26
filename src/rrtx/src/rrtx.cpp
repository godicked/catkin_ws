#include <rrtx/rrtx.hpp>
#include <math.h>

#include <visualization_msgs/Marker.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

using namespace std;
using namespace boost;

const double infinity = numeric_limits<double>::infinity();

boost::random::mt19937 gen;


namespace rrt
{
    
    RRTx::RRTx()
    {
        marker_pub  = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 1000);

        nh_.param<string>("map_frame", this->map_frame, "/map");
    }

    RRTx::RRTx(costmap_2d::Costmap2D *costmap)
    {
        RRTx();
        setCostmap(costmap);
    }

    void RRTx::setCostmap(costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
    }
    
    void RRTx::addVertex(Node *v)
    {
        point v_point(v->x, v->y);
        rtree.insert(make_pair(v_point, v));
    }

    vector<Node *> RRTx::inN(Node v)
    {
        vector<Node *> in = v.inNz;
        in.insert(in.end(), v.inNr.begin(), v.inNr.end());
        return in;
    }

    vector<Node *> RRTx::outN(Node v)
    {
        vector<Node *> out = v.outNz;
        out.insert(out.end(), v.outNr.begin(), v.outNr.end());
        return out;
    }

    void RRTx::removeN(vector<rrt::Node *> *vec, rrt::Node *u)
    {
        auto it = vec->begin();
        while (it != vec->end())
        {
            if (*it == u)
            {
                vec->erase(it);
                return;
            }

            it++;
        }
    }

    Node *RRTx::nearest(Node v)
    {
        vector<Leaf> result;
        point p(v.x, v.y);

        rtree.query(bgi::nearest(p, 2), back_inserter(result));

        if(result[0].second != vbot_)
            return result[0].second;
        return result[1].second; 
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
            Node *node  = search[i].second;
            float dist  = distance(*node, v);

            if(dist > radius)
            {
                continue;
            }

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

        v.x = u.x + dx * (radius - 0.001);
        v.y = u.y + dy * (radius - 0.001);

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
        for(double i = 0; i < dist - resolution; i += resolution)
        {
            double wx = v.x + dx * (i / dist);
            double wy = v.y + dy * (i / dist);
           
           unsigned int mx, my;
            costmap_->worldToMap(wx, wy, mx, my);
            unsigned char cost = costmap_->getCost(mx, my);
            
            if(cost > 50)
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

        if(cost > 100)
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

        //cout << "near" << endl;
        NearInfoVec vnear = near(*v);
        //cout << "findParent" << endl;
        //cout << "aaahhhh" << endl;
        findParent(v, vnear);
        if(v->parent == nullptr)
            return;

        //cout << "addVertex" << endl;
        addVertex(v);
        v->parent->childs.push_back(v);

        //cout << "for nver : vnear" << endl;
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
        //cout << "verify queue" << endl;
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
                (queue.top()->key < vbot_->key ||
                abs(vbot_->lmc - vbot_->g) > 0.00001 ||
                vbot_->g == infinity ||
                queueContains(vbot_)))
        {

            //Take first Node from queue and remove it
            Node *v = queuePop();

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
            if(u->parent != v)
                continue;

            double dist = distance(*v, *u) + u->lmc;
            if(v->lmc > dist)
            {
                p       = u;
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

     Node *RRTx::queuePop()
     {
         Node *toRemove = queue.top();
         hash.erase(toRemove);
         queue.pop();
         return toRemove;
     }

     void RRTx::updateKey(Node *v)
     {
         v->key.k1 = min(v->g, v->lmc);
         v->key.k2 = v->g;
     }

     void RRTx::updateRadius()
     {
         int    n       = rtree.size();
         double term1   = (y / M_PI) * (log(n) / n);
         radius  = min( pow(term1, 0.5), maxDist);

         //cout << "new radius: " << radius << endl;
     }


     Node RRTx::randomNode()
     {
         Node node;
         
         boost::random::uniform_int_distribution<> xr(0, costmap_->getSizeInCellsX() -1);
         boost::random::uniform_int_distribution<> yr(0, costmap_->getSizeInCellsY() -1);

         int x = xr(gen);
         int y = yr(gen);

         costmap_->mapToWorld(x, y, node.x, node.y);
         
         return node;
     }

     Node RRTx::rootNode()
     {
         //cout << "root node" << endl;
         return *goal_;
     }

     RRTx::NodeContainer RRTx::getContainer()
     {
         return nodeContainer;
     }

     void RRTx::grow()
     {
         
         updateRadius();

         Node new_v     = randomNode();
         Node *vnearest = nearest(new_v);

         if(distance(new_v, *vnearest) > radius)
             new_v = saturate(new_v, *vnearest);


         if(!isObstacle(new_v))
         {
             nodeContainer.push_back(new_v);
             Node *v = &nodeContainer.back();
           
             extend(v);
             if(v->parent != nullptr)
             {
                rewireNeighbors(v);
                reduceInconsist();
             }
             else
             {
                 nodeContainer.pop_back();
             }
         }
     }

     void RRTx::grow(unsigned int iteration)
     {
         ros::Time t = ros::Time::now();
         for(int i = 0; i < iteration; i++)
             grow();
         ros::Duration d = ros::Time::now() - t;
         cout << d.toSec() << endl;
         cout << nodeContainer.size() << endl;
     }

     void RRTx::init(geometry_msgs::Pose start, geometry_msgs::Pose goal)
     {
         init(start.position.x, start.position.y, goal.position.x, goal.position.y);
     }

     void RRTx::init(double sx, double sy, double gx, double gy)
     {
         ROS_INFO("Init RRTx.  start: %.1f:%.1f, goal: %.1f:%.1f", sx, sy, gx, gy);
         ROS_INFO("Costmap size: %.1f:%.1f, costmap resolution %.2f", costmap_->getSizeInMetersX(), 
         costmap_->getSizeInMetersY(), costmap_->getResolution());

         rtree.clear();
         queue.clear();
         hash.clear();
         nodeContainer.clear();
         
         //gen = boost::random::mt19937(time(0));
         
         Node start;
         start.x = sx;
         start.y = sy;

         Node goal;
         goal.x     = gx;
         goal.y     = gy;
         goal.g     = 0;
         goal.lmc   = 0;

         
         nodeContainer.push_back(start);
         vbot_ = &nodeContainer.back();
         addVertex(vbot_);

         nodeContainer.push_back(goal);
         goal_ = &nodeContainer.back();
         addVertex(goal_);

     }

     bool RRTx::getPath(Path &path)
     {
         Node *v = vbot_;
         bool foundGoal = false;

         do
         {
            if(v == goal_) {
                foundGoal = true;
            }

            geometry_msgs::Pose p;
            p.position.x = v->x;
            p.position.y = v->y;

            p.orientation.x = 0;
            p.orientation.y = 0;
            p.orientation.z = 0;
            p.orientation.w = 1;

            path.push_back(p);

            v = v->parent;

         }  while(v != nullptr);

         return  foundGoal;
     }

     void RRTx::setMaxDist(double dist)
     {
         maxDist = dist;
         // As defined in RRT* Y >= 2 * dim + (1 + 1/dim) * u(XFree)
         // u = volume, XFree = free space. y > Y since we use total volume of map
         y = 6.0 * costmap_->getSizeInMetersX() * costmap_->getSizeInMetersY();
     }

     void RRTx::publish(bool path, bool tree)
     {
         // Publish Goal
         visualization_msgs::Marker goal;
         goal.header.frame_id       = map_frame;
         goal.header.stamp          = ros::Time::now();
         goal.ns                    = "rrtx";
         goal.id                    = 0;
         goal.action                = visualization_msgs::Marker::ADD;
         goal.type                  = visualization_msgs::Marker::POINTS;
         
         goal.pose.orientation.w    = 1.0;
         goal.scale.x               = 0.05;
         goal.scale.y               = 0.05;
         
         goal.color.a               = 1;
         goal.color.r               = 1;

         geometry_msgs::Point pgoal;
         pgoal.x = goal_->x;
         pgoal.y = goal_->y;
         pgoal.z = 1;
         goal.points.push_back(pgoal);

         marker_pub.publish(goal);

         // publish Start (vbot)
         visualization_msgs::Marker vbot;
         vbot.header.frame_id       = map_frame;
         vbot.header.stamp          = ros::Time::now();
         vbot.ns                    = "rrtx";
         vbot.id                    = 1;
         vbot.action                = visualization_msgs::Marker::ADD;
         vbot.type                  = visualization_msgs::Marker::POINTS;
         
         vbot.pose.orientation.w    = 1.0;
         vbot.scale.x               = 0.05;
         vbot.scale.y               = 0.05;
         
         vbot.color.a               = 1;
         vbot.color.r               = 1;
         vbot.color.g               = 1;

         geometry_msgs::Point pvbot;
         pvbot.x = vbot_->x;
         pvbot.y = vbot_->y;
         pvbot.z = 1;
         vbot.points.push_back(pvbot);

         marker_pub.publish(vbot);

         if(path)
         {
             visualization_msgs::Marker nodes, edges;

             nodes.header.frame_id  = map_frame;
             nodes.header.stamp     = ros::Time::now();
             nodes.ns               = "rrtx";
             nodes.id               = 4; 
             nodes.action           = visualization_msgs::Marker::ADD;
             nodes.type             = visualization_msgs::Marker::POINTS;

             nodes.scale.x          = 0.02;
             nodes.scale.y          = 0.02;

             nodes.color.a          = 1;
             nodes.color.r          = 1;
             nodes.color.g          = 0.5;

             
             edges.header.frame_id  = map_frame;
             edges.header.stamp     = ros::Time::now();
             edges.ns               = "rrtx";
             edges.id               = 5; 
             edges.action           = visualization_msgs::Marker::ADD;
             edges.type             = visualization_msgs::Marker::LINE_LIST;

             edges.scale.x          = 0.01;
             edges.scale.y          = 0.01;

             edges.color.a          = 1;
             edges.color.r          = 0.5;
             edges.color.g          = 1;

             Node *node = vbot_;
             do
             {
                 geometry_msgs::Point p;
                 p.x = node->x;
                 p.y = node->y;

                 nodes.points.push_back(p);

                 if (node->parent != nullptr)
                 {
                     edges.points.push_back(p);
                     p.x = node->parent->x;
                     p.y = node->parent->y;
                     edges.points.push_back(p);
                 }

                 node = node->parent;
             }  while(node != nullptr);

             marker_pub.publish(nodes);
             marker_pub.publish(edges);
         }

         if(tree)
         {
             visualization_msgs::Marker nodes, edges;

             nodes.header.frame_id  = map_frame;
             nodes.header.stamp     = ros::Time::now();
             nodes.ns               = "rrtx";
             nodes.id               = 2; 
             nodes.action           = visualization_msgs::Marker::ADD;
             nodes.type             = visualization_msgs::Marker::POINTS;

             nodes.scale.x          = 0.05;
             nodes.scale.y          = 0.05;

             nodes.color.a          = 1;
             nodes.color.b          = 1;

             
             edges.header.frame_id  = map_frame;
             edges.header.stamp     = ros::Time::now();
             edges.ns               = "rrtx";
             edges.id               = 3; 
             edges.action           = visualization_msgs::Marker::ADD;
             edges.type             = visualization_msgs::Marker::LINE_LIST;

             edges.scale.x          = 0.02;
             edges.scale.y          = 0.02;

             edges.color.a          = 1;
             edges.color.g          = 1;

             for (auto node : nodeContainer)
             {
                 geometry_msgs::Point p;
                 p.x = node.x;
                 p.y = node.y;

                 nodes.points.push_back(p);

                 if (node.parent == nullptr)
                     continue;

                 edges.points.push_back(p);
                 p.x = node.parent->x;
                 p.y = node.parent->y;
                 edges.points.push_back(p);
             }

             marker_pub.publish(nodes);
             marker_pub.publish(edges);
         }
     }

};


typedef rrt::RRTx::NodeContainer NodeContainer;


/*
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
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrtx");

    costmap_2d::Costmap2D costmap(1000, 1000, 0.1, 0, 0);
    
    for(int i = 0; i < 200; i++) { for(int j = 0; j < 400; j++) {
        costmap.setCost(i + 200, j + 100, 255);
        costmap.setCost(j + 500, i + 790, 255);
        costmap.setCost(i + 600, j + 300, 255);
    }}


    rrt::RRTx rrt(&costmap);
    rrt.setMaxDist(15);
    rrt.init(20,20, 45,70); 
    rrt.grow(1000);

    ros::Rate rate(1);
    while(ros::ok())
    {
        //cout << "loop" << endl;
        rrt.publish(false, true);

        rate.sleep();
    }

    

    return 0;
}




