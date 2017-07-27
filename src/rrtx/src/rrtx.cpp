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

    double dist(geometry_msgs::Pose a, Node b)
    {
        double dx = a.position.x - b.x;
        double dy = a.position.y - b.y;

        return sqrt(dx*dx + dy*dy);
    }
    
    RRTx::RRTx()
    {
        marker_pub  = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 1000);
        nh_.param<string>("map_frame", this->map_frame, "/map");
        publisher.initialize(&nh_, map_frame);
    }

    RRTx::RRTx(costmap_2d::Costmap2D *costmap) : RRTx()
    {
        setCostmap(costmap);
    }

    void RRTx::setCostmap(costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
    }

    void RRTx::setConstraint(double steering_angle, double wheelbase)
    {
        builder = KinematicPathBuilder(nh_, wheelbase, steering_angle);
        minDist = wheelbase;
        constraint = true;
    }
    
    void RRTx::addVertex(Node *v)
    {
        point v_point(v->x, v->y);
        RTreePoint leaf = make_pair(v_point, v);
        rtree.insert(leaf);
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
        vector<RTreePoint> result;
        point p(v.x, v.y);

        rtree.query(bgi::nearest(p, 2), back_inserter(result));

        if(result[0].second != vbot_->parent)
            return result[0].second;
        return result[1].second; 
    }

    vector<Node *> RRTx::near(Node v)
    {
        vector<RTreePoint> search;
        point p1(v.x - radius, v.y - radius), 
              p2(v.x + radius, v.y + radius);

        box b(p1, p2);

        vector<Node *> nodes;
        rtree.query(bgi::intersects(b), back_inserter(search));
        for(int i = 0; i < search.size(); i++)
        {
            Node *node  = search[i].second;
            float dist  = distance(*node, v);

            if(dist > radius)
            {
                continue;
            }

            nodes.push_back(node);
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
        if(dist > radius)
        {
            double dx = (v.x - u.x) / dist;
            double dy = (v.y - u.y) / dist;

            v.x = u.x + dx * (radius - 0.001);
            v.y = u.y + dy * (radius - 0.001);
        }

        return v;
    }

    void RRTx::findParent(Node *v, vector<Node *> nodes)
    {
        for(auto u : nodes)
        {
            double dist = distance(*v, *u);
            
            if(v->lmc > dist + u->lmc)
            {
                v->parent    = u;
                v->lmc       = dist + u->lmc;
                
            }
        }
        
    }

    double RRTx::cost(Node *a, Node *b)
    {
        return trajectories[make_pair(a,b)].cost;
    }

    void RRTx::setCost(Node *a, Node *b, double cost)
    {
        trajectories[make_pair(a,b)].cost = cost;
    }

    Trajectory *RRTx::trajectory(Node *a, Node *b)
    {
        return &trajectories[make_pair(a, b)];
    }

    vector<Node *> RRTx::findTrajectories(Node *v, vector<Node *> nodes)
    {
        vector<Node *> exist;
        for(auto u : nodes)
        {
            if(trajectoryExist(v, u))
            {
                makeTrajectory(v, u);
                exist.push_back(u);
            }
        }
        return exist;
    }

    bool RRTx::trajectoryExist(Node *v, Node *u)
    {
        double dist = distance(*v, *u);

        double dx = u->x - v->x;
        double dy = u->y - v->y;

        double resolution = costmap_->getResolution();
        for(double i = 0; i < dist - resolution; i += resolution)
        {
            double wx = v->x + dx * (i / dist);
            double wy = v->y + dy * (i / dist);
           
           unsigned int mx, my;
            costmap_->worldToMap(wx, wy, mx, my);
            //ROS_INFO("test %d,%d", mx, my);
            unsigned char cost = costmap_->getCost(mx, my);
            
            if(cost > 50)
                return false;
        }
        
        return true;
    }

    Trajectory *RRTx::makeTrajectory(Node *v, Node *u)
    {
        double dist = distance(*v, *u);
        trajectories[make_pair(v,u)] = Trajectory(u, v, dist);
        return &trajectories[make_pair(v, u)];
    }

    bool RRTx::isOutOfBound(unsigned int mx, unsigned int my)
    {
        return  mx < 0 || my < 0 ||
                mx >= costmap_->getSizeInCellsX() ||
                my >= costmap_->getSizeInCellsY();
    }

    bool RRTx::isObstacle(Node v)
    {
        unsigned int mx, my;
        costmap_->worldToMap(v.x, v.y, mx, my);

        //if(isOutOfBound(mx, my))
        //    return true;

        unsigned char cost = costmap_->getCost(mx, my);

        return cost > 150;
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
        vector<Node *> nearNodes = near(*v);
        vector<Node *> validNodes = findTrajectories(v, nearNodes);

        //cout << "findParent" << endl;
        findParent(v, validNodes);
        if(v->parent == nullptr)
            return;

        //cout << "addVertex" << endl;
        addVertex(v);
        v->parent->childs.push_back(v);

        //cout << "for nver : vnear" << endl;
        for(auto u : validNodes)
        {

            v->outNz.push_back(u);
            u->inNr.push_back(v);

            // Inverse trajectory u to v.

            u->outNr.push_back(v);
            v->inNz.push_back(u);
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

                double dist = cost(u, v) + v->lmc;
                if(u->lmc > dist)
                {
                    u->lmc      = dist;
                    makeParentOf(v, u);

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
            if(u->parent == v)
                continue;

            double dist = cost(v, u) + u->lmc;
            if(v->lmc > dist)
            {
                p = u;
            }
        }
        makeParentOf(p, v);
    }

	void RRTx::queueInsert(Node *v)
	 {
	    updateKey(v); 
	    nodeHash[v] = queue.push(v);
	 }
	 
	 bool RRTx::queueContains(Node *v)
	 {
	    return nodeHash.find(v) != nodeHash.end();
	 }
	 
	 void RRTx::queueUpdate(Node *v)
	 {
	    updateKey(v);
	    queue.update(nodeHash[v]);
	 }
	 
	 void RRTx::queueRemove(Node *v)
	 {
	    queue.erase(nodeHash[v]);
	    nodeHash.erase(v);
	 }

     Node *RRTx::queuePop()
     {
         Node *toRemove = queue.top();
         nodeHash.erase(toRemove);
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
         while(nodeContainer.size() < iteration)
             grow();
         ros::Duration d = ros::Time::now() - t;
         ROS_INFO("RRTx grow took %.2f seconds ( %ld nodes )", d.toSec(), nodeContainer.size());
     }

     void RRTx::init(geometry_msgs::Pose start, geometry_msgs::Pose goal)
     {
         tf::Pose pose;
         tf::poseMsgToTF(start, pose);
         double yaw_angle = tf::getYaw(pose.getRotation());

         init(start.position.x, start.position.y, yaw_angle, goal.position.x, goal.position.y);
         lastPose = start;
     }

     void RRTx::init(double sx, double sy, double stheta, double gx, double gy)
     {
         ROS_INFO("Init RRTx.  start: %.1f:%.1f, goal: %.1f:%.1f", sx, sy, gx, gy);
         ROS_INFO("Costmap size: %.1f:%.1f, costmap resolution %.2f", costmap_->getSizeInMetersX(), 
         costmap_->getSizeInMetersY(), costmap_->getResolution());

         rtree.clear();
         queue.clear();
         nodeHash.clear();
         nodeContainer.clear();
         trajectories.clear();
         infTrajectories.clear();
         orphanHash.clear();
         
         //gen = boost::random::mt19937(time(0));
         
         vbot_theta = stheta;
         Node vbot;
         vbot.x = sx;
         vbot.y = sy;

         Node start;
         start.x = vbot.x + cos(stheta) * minDist;
         start.y = vbot.y + sin(stheta) * minDist;


         Node goal;
         goal.x     = gx;
         goal.y     = gy;
         goal.g     = 0;
         goal.lmc   = 0;

         if(isObstacle(goal))
            ROS_WARN("goal is obstacle");

         nodeContainer.push_back(vbot);
         vbot_ = &nodeContainer.back();
         //addVertex(vbot_);

         nodeContainer.push_back(start);
         Node * start_ = &nodeContainer.back();
         addVertex(start_);
         vbot_->parent = start_;

         nodeContainer.push_back(goal);
         goal_ = &nodeContainer.back();
         addVertex(goal_);

     }

     // get signed angle in range ]-pi; pi]
     double RRTx::getAngle(Node a, Node b, Node c)
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

     void RRTx::updateTree(double origin_x, double origin_y, double size_x, double size_y)
     {
         ros::Time time = ros::Time::now();
         //ROS_INFO("start update tree");
         //findFreeTrajectories(map);
         reduceInconsist();

         addObstacle(origin_x, origin_y, size_x, size_y);
         //ROS_INFO("end add obstacle");
         propogateDescendants();
         //verrifyQueue(vbot_->parent);
         reduceInconsist();
         //ROS_INFO("end update");

         ros::Duration d = ros::Time::now() - time;
         //ROS_INFO("update Tree took %.3fsec", d.toSec());

         if(lastPath.size() > 1)
         {
             makeParentOf(lastPath[1], vbot_);
         }
     }

     void RRTx::verrifyOrphan(Node *v)
     {
         if(queueContains(v))
         {
             queueRemove(v);
         }

         orphanHash[v] = true;
     }

     void RRTx::insertOrphanChildren(Node *v)
     {
        orphanHash[v] = true;

        for(auto c : v->childs)
        {
            insertOrphanChildren(c);
        }
     }

     void RRTx::propogateDescendants()
     {
         //ROS_INFO("propagateDescendants");
         // get all orphan nodes
         vector<Node *> orphans;
         orphans.reserve(orphanHash.size());
         for(auto o : orphanHash) 
         { 
             orphans.push_back(o.first);
         }
         //ROS_INFO("%ld orphans", orphans.size());
         // insert children to orphans
         for(auto v : orphans)
         {
             insertOrphanChildren(v);
         }
         // get new list of orphan nodes
         orphans.clear();
         orphans.reserve(orphanHash.size());
         for(auto v : orphanHash) 
         { 
             orphans.push_back(v.first);
         }
         
        //  ROS_INFO("%ld orphans", orphans.size());
         // verifiy outgoing edges
         for(auto v : orphans)
         {
             vector<Node *> neighbors = outN(*v);
             neighbors.push_back(v);
             for(auto n : neighbors)
             {
                 if(orphanHash[n]) continue;

                 n->g = infinity;
                 verrifyQueue(n);
             }

         }

         for (auto v : orphans)
         {
             v->g = infinity;
             v->lmc = infinity;
             if (v->parent)
             {
                auto &childs = v->parent->childs;
                childs.erase(remove_if(
                    childs.begin(), childs.end(), [v](Node *c) { return v == c; }),
                    childs.end());
                v->parent = nullptr;
             }
         }

         orphanHash.clear();
        //  ROS_INFO("end propagate");
     }
     void RRTx::findFreeTrajectories(costmap_2d::Costmap2D map)
     {

     }

     void RRTx::addObstacle(double origin_x, double origin_y, double size_x, double size_y)
     {
         vector<Trajectory *> hit;
         vector<RTreePoint> search;
         box b(point(origin_x, origin_y), point(origin_x + size_x, origin_y + size_y));
         rtree.query(bgi::intersects(b), back_inserter(search));

        //  ROS_INFO("found %ld nodes", search.size());

         for(auto s : search)
         {
            Node *v = s.second;

            for(auto n : outN(*v))
            {
                if(cost(v, n) == infinity) continue;
                if(!trajectoryExist(v, n))
                {
                    setCost(v, n, infinity);
                    hit.push_back(trajectory(v, n));
                }
            }
         }

        //  ROS_INFO("hit %ld trajectories", hit.size());

         for(auto traj : hit)
         {
            infTrajectories.push_back(traj);
            if(traj->source->parent == traj->target)
            {
                //ROS_INFO("is parent!");
                //infTrajectories.push_back(traj);
                verrifyOrphan(traj->source);
            }
            if(traj->target->parent == traj->source)
            {
                //ROS_INFO("is parent!");
                //infTrajectories.push_back(traj);
                verrifyOrphan(traj->target);
            }
         }
     }

  
     bool RRTx::computePath(Path &path)
     {
         
         ros::Time t = ros::Time::now();
         vector<Node *> nodes;

         if(constraint)
         {
            nodes = builder.buildPath(vbot_, goal_, &trajectories);
            builder.publishVisited();
            // rewire parent so the path gets recalculated on obstacle change
            for(int i = 1; i < nodes.size() -1; i++)
            {
                makeParentOf(nodes[i+1], nodes[i]);
            }
         }
         else
         {
            Node *v = vbot_;
            // follow optimal path
            while(v != nullptr)
            {
                nodes.push_back(v);
                v = v->parent;
            }
         }
         
         for(auto node : nodes)
         {
            geometry_msgs::Pose p;
            p.position.x = node->x;
            p.position.y = node->y;

            p.orientation.x = 0;
            p.orientation.y = 0;
            p.orientation.z = 0;
            p.orientation.w = 1;

            path.push_back(p);
         }

         ros::Duration d = ros::Time::now() - t;
         //ROS_INFO("Get Path took %.2f seconds", d.toSec());

         lastPath = nodes;  

         return  true;
     }

    void RRTx::updateRobot(geometry_msgs::Pose robot)
    {
        Node vbot;
        vbot.x = robot.position.x;
        vbot.y = robot.position.y;

        if(lastPath.size() > 3)
        {
            if(distance(vbot, *lastPath[1]) > distance(vbot, *lastPath[2]) && cost(lastPath[1], lastPath[2]) != infinity)
            {
                vbot_ = lastPath[1];
                makeParentOf(lastPath[2], vbot_);
                updateKey(vbot_);
            }
        }
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
        
        publisher.publishTree(vbot_, goal_, nodeContainer, true);
        publisher.publishInfTrajectories(infTrajectories);
        return;

    }

    /*
    *   Make set the parent of the node v. Also actualise childs from old and new parent
    */ 
    void RRTx::makeParentOf(Node *parent, Node *v)
    {
        if(v->parent)
        {
            auto& childs = v->parent->childs;
            childs.erase(remove_if(
                childs.begin(), childs.end(), [v](Node *c){ return v == c; }),
                childs.end());
        }

        v->parent = parent;
        if(v->parent)
        {
            parent->childs.push_back(v);
        }
    }

}; // namespace rrt