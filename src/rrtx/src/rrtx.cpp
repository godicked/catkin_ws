#include <rrtx/rrtx.hpp>
#include <math.h>

#include <visualization_msgs/Marker.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <ompl/base/goals/GoalState.h>


using namespace std;
using namespace boost;
using namespace ompl;
using namespace ompl::base;

const double infinity = numeric_limits<double>::infinity();

boost::random::mt19937 gen;



namespace rrt
{

    void print(Node *v)
    {
        double x, y;
        x = v->state->as<SE2StateSpace::StateType>()->getX();
        y = v->state->as<SE2StateSpace::StateType>()->getY();

        cout << x << " : " << y << endl;
    }
    
    RRTx::RRTx()    // double dist(geometry_msgs::Pose a, Node b)
    // {
    //     double dx = a.position.x - b.x;
    //     double dy = a.position.y - b.y;

    //     return sqrt(dx*dx + dy*dy);
    // }
    {
        marker_pub  = nh_.advertise<visualization_msgs::Marker>("rrt_tree", 1000);
        nh_.param<string>("map_frame", this->map_frame, "/map");
        //publisher.initialize(&nh_, map_frame);
    }

    // RRTx::RRTx(costmap_2d::Costmap2D *costmap) : RRTx()
    // {
    //     setCostmap(costmap);
    // }

    RRTx::RRTx(SpaceInformationPtr si) : si_(si)
    {
        nn_.reset( new ompl::NearestNeighborsGNAT<Node *>() );
        nn_->setDistanceFunction([this](const Node *v, const Node *u) {
            return si_->distance(v->state, u->state);
        });

        sampler_ = si_->allocStateSampler();
    }

    // void RRTx::setCostmap(costmap_2d::Costmap2D *costmap)
    // {
    //     costmap_ = costmap;
    // }

    void RRTx::setConstraint(double steering_angle, double wheelbase)
    {
        // builder = KinematicPathBuilder(nh_, wheelbase, steering_angle);
        // minDist = wheelbase;
        // constraint = true;
    }
    
    void RRTx::addVertex(Node *v)
    {
        nn_->add(v);
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
        return nn_->nearest(&v);
    }

    vector<Node *> RRTx::near(Node v)
    {
        vector<Node *> nodes;
        // cout << "radius is " << radius << endl;
        nn_->nearestR(&v, radius+0.001, nodes);
        return nodes;
    }

    double RRTx::distance(Node v, Node u)
    {
        return si_->distance(v.state, u.state);
    }

    Node RRTx::saturate(Node v, Node u)
    {
        double dist = distance(v, u);
        if(dist > maxDist)
        {
            si_->getStateSpace()->interpolate(u.state, v.state, maxDist / dist, v.state);
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
        // cout << "find in: " << nodes.size() << endl;
        vector<Node *> exist;

        // add start Node if v is near. vbot_ is not stored in the NearestNeigbhors datastructure
        if(distance(*v, *vbot_) <= radius)
        {
            nodes.push_back(vbot_);
        }

        for(auto u : nodes)
        {
            if(trajectoryExist(v, u))
            {
                // cout << "exist" << endl;
                makeTrajectory(v, u);
                exist.push_back(u);
            }
        }

        return exist;
    }

    bool RRTx::trajectoryExist(Node *v, Node *u)
    {
        return si_->checkMotion(v->state, u->state);
    }

    Trajectory *RRTx::makeTrajectory(Node *v, Node *u)
    {
        double dist = distance(*v, *u);
        trajectories[make_pair(v,u)] = Trajectory(u, v, dist);
        return &trajectories[make_pair(v, u)];
    }

    // bool RRTx::isOutOfBound(unsigned int mx, unsigned int my)
    // {
    //     return  mx < 0 || my < 0 ||
    //             mx >= costmap_->getSizeInCellsX() ||
    //             my >= costmap_->getvector<RTreePoint> result;
    //     point p(v.x, v.y);
    // }

    bool RRTx::isObstacle(Node v)
    {
        return !si_->isValid(v.state);
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
        //cout << "full size " << nn_->size() << endl;

        vector<Node *> nearNodes = near(*v);
        vector<Node *> validNodes = findTrajectories(v, nearNodes);

        // cout << "found " << validNodes.size() << endl;
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
         // at start nn_ size is 1, so we add one to avoid 0 radius
         int    n       = nn_->size() + 1;
         double term1   = (y / M_PI) * (log(n) / n);
         radius  = min( pow(term1, 0.5), maxDist);

         //cout << "new radius: " << radius << endl;
     }

     /* randomNode
     ** returns a randomly sampled Node */
     Node RRTx::randomNode()
     {
         Node node;
         node.state = si_->allocState();
         sampler_->sampleUniform(node.state);

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
         //cout << "grow" << endl;
         updateRadius();

         Node new_v     = randomNode();
        //   cout << endl;
        //  print(&new_v);
         Node *vnearest = nearest(new_v);
         new_v = saturate(new_v, *vnearest);
         
        //  print(&new_v);

         if(!isObstacle(new_v))
         {
             nodeContainer.push_back(new_v);
             Node *v = &nodeContainer.back();
             
             extend(v);
             if(v->parent != nullptr)
             {
                // cout << "add" << endl;
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

     void RRTx::init(ProblemDefinitionPtr pdef)
     {

        pdef_ = pdef;

        // ROS_INFO("Init RRTx.  start: %.1f:%.1f, goal: %.1f:%.1f", sx, sy, gx, gy);
        // ROS_INFO("Costmap size: %.1f:%.1f, costmap resolution %.2f", costmap_->getSizeInMetersX(), 
        // costmap_->getSizeInMetersY(), costmap_->getResolution());

        queue.clear();
        nodeHash.clear();
        nodeContainer.clear();
        trajectories.clear();
        // infTrajectories.clear();
        orphanHash.clear();
        
        //gen = boost::random::mt19937(time(0));
        
        Node vbot;
        vbot.state = pdef_->getStartState(0);

        Node goal;
        goal.state = pdef_->getGoal()->as<GoalState>()->getState();
        goal.g     = 0;
        goal.lmc   = 0;

        if(isObstacle(goal))
        ROS_WARN("goal is obstacle");

        nodeContainer.push_back(vbot);
        vbot_ = &nodeContainer.back();

        nodeContainer.push_back(goal);
        goal_ = &nodeContainer.back();
        addVertex(goal_);

     }

     // get signed angle in range ]-pi; pi]
    //  double RRTx::getAngle(Node a, Node b, Node c)
    //  {
    //     double x1, y1, x2, y2;

    //     x1 = b.x - a.x;
    //     y1 = b.y - a.y;

    //     x2 = b.x - c.x;
    //     y2 = b.y - c.y;

    //     double dot = x1*x2 + y1*y2;
    //     double cross = x1 * y2 - y1 * x2;

    //     double angle = atan2(cross, dot);

    //     return angle;
    //  }

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
    //     vector<Trajectory *> hit;
    //     vector<RTreePoint> search;
    //     box b(point(origin_x, origin_y), point(origin_x + size_x, origin_y + size_y));
    //     rtree.query(bgi::intersects(b), back_inserter(search));

    // //  ROS_INFO("found %ld nodes", search.size());

    //     for(auto s : search)
    //     {
    //     Node *v = s.second;

    //     for(auto n : outN(*v))
    //     {
    //         if(cost(v, n) == infinity) continue;
    //         if(!trajectoryExist(v, n))
    //         {
    //             setCost(v, n, infinity);
    //             hit.push_back(trajectory(v, n));
    //         }
    //     }
    //     }

    // //  ROS_INFO("hit %ld trajectories", hit.size());

    //     for(auto traj : hit)
    //     {
    //     infTrajectories.push_back(traj);
    //     if(traj->source->parent == traj->target)
    //     {
    //         //ROS_INFO("is parent!");
    //         //infTrajectories.push_back(traj);
    //         verrifyOrphan(traj->source);
    //     }
    //     if(traj->target->parent == traj->source)
    //     {
    //         //ROS_INFO("is parent!");
    //         //infTrajectories.push_back(traj);
    //         verrifyOrphan(traj->target);
    //     }
    //     }
    }

  
    bool RRTx::computePath(Path &path)
    {
        
        ros::Time t = ros::Time::now();
        vector<Node *> nodes;

        Node *v = vbot_;
        //follow optimal path
        while(v != nullptr)
        {
            nodes.push_back(v);
            v = v->parent;
        }
        
        for(auto node : nodes)
        {
            // geometry_msgs::Pose p;
            // p.position.x = node->x;
            // p.position.y = node->y;

            // p.orientation.x = 0;
            // p.orientation.y = 0;
            // p.orientation.z = 0;
            // p.orientation.w = 1;

            // path.push_back(p);
        }

        ros::Duration d = ros::Time::now() - t;
        //ROS_INFO("Get Path took %.2f seconds", d.toSec());

        lastPath = nodes;  

        return  true;
    }

    void RRTx::updateRobot(geometry_msgs::Pose robot)
    {
        // Node vbot;
        // vbot.x = robot.position.x;
        // vbot.y = robot.position.y;

        // if(lastPath.size() > 3)
        // {
        //     if(distance(vbot, *lastPath[1]) > distance(vbot, *lastPath[2]) && cost(lastPath[1], lastPath[2]) != infinity)
        //     {
        //         vbot_ = lastPath[1];
        //         makeParentOf(lastPath[2], vbot_);
        //         updateKey(vbot_);
        //     }
        // }
    }

    void RRTx::setMaxDist(double dist)
    {
        maxDist = dist;
        // As defined in RRT* Y >= 2 * dim + (1 + 1/dim) * u(XFree)
        // u = volume, XFree = free space. y > Y since we use total volume of map
        y = 6.0 * si_->getSpaceMeasure();
    }

    void RRTx::publish(bool path, bool tree)
    {
        
        publisher.publishTree(vbot_, goal_, nodeContainer, true);
        // publisher.publishInfTrajectories(infTrajectories);
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