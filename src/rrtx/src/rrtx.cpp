#include <rrtx/RRTx.hpp>

#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/GeometricEquations.h>


#include <random>

#include <ros/ros.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace boost;
using namespace ompl;
using namespace ompl::base;



std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)


namespace rrt
{

    RRTx::RRTx(SpaceInformationPtr si) : Planner(si, "RRTx")
    {
        nn_.reset( new ompl::NearestNeighborsGNAT<Motion *>() );
        nn_->setDistanceFunction([this](const Motion *v, const Motion *u) {
            return si_->distance(v->state, u->state);
        });
    }

    void RRTx::setup()
    {
        if(setup_) return;
        Planner::setup();
        // free memory before allocating new space
        clear();

        sampler_ = si_->allocStateSampler();

        setup_ = true;

        if(!pdef_)
        {
            cout << "No problem definition" << endl;
        }
        opt_ = pdef_->getOptimizationObjective();

        vbot_ = new Motion();
        vbot_->state = pdef_->getStartState(0);
        vbot_->g = opt_->infiniteCost();
        vbot_->lmc = opt_->infiniteCost();

        goal_ = new Motion();
        goal_->state = pdef_->getGoal()->as<GoalState>()->getState();
        goal_->g     = opt_->identityCost(); // 0.0
        goal_->lmc   = opt_->identityCost(); // 0.0

        nn_->add(goal_);

        // As defined in RRT* Y >= 2(1 + 1/dim)^(1/dim) * u(XFree)
        // u = volume, XFree = free space. y > Y since we use total volume of map
        // const double dim = si_->getStateDimension();
        const double dim = si_->getStateDimension() + 0.0;
        const double free = si_->getSpaceMeasure();

        y_ = std::pow(2*(1 + 1 / dim), 1 / dim) * std::pow(free / unitNBallMeasure(dim), 1 / dim);

        symmetric_ = si_->getStateSpace()->hasSymmetricDistance();
        // if(symmetric_) cout << "SYMMETRIC" << endl;
    }

    PlannerStatus RRTx::solve(const PlannerTerminationCondition &ptc)
    {
        // setup
        checkValidity();

        cout << "start with maxDist " << maxDist_ << endl;

        vector<double> costs;
        vector<ros::Duration> times;
        vector<double> radius;

        auto time = ros::Time::now();

        while(!ptc)
        {
            iteration_++;
            grow();
            
            costs.push_back(vbot_->lmc.value());
            times.push_back(ros::Time::now() - time);
            radius.push_back(radius_);
        }

        ofstream myfile;
        myfile.open ("/home/godicke/BachelorArbeit/benchmark.txt");

        if( !myfile )
        {
           cout << "Couldn't open file."  << endl;
        }
        
        for(int i = 0; i < costs.size(); i++)
        {
            myfile << i << "," << times[i].toSec() << "," << costs[i] << "," << radius[i] << "\n";
        }
        myfile.close();

        computePath();

        bool solved = !opt_->isCostEquivalentTo(vbot_->lmc, opt_->infiniteCost());
        return PlannerStatus(solved, false);
    }

    void RRTx::computePath()
    {   
        vector<Motion *> mpath;
        // follow parent Motion
        for(Motion *v = vbot_; v != nullptr; v = v->parent)
        {
            mpath.push_back(v);
        }


        // fill path information
        std::shared_ptr<geometric::PathGeometric> path( new geometric::PathGeometric(si_) );
        for(auto v : mpath)
        {
            path->append(v->state);
        }
        
        PlannerSolution solution(path);
        solution.setPlannerName(getName());
        pdef_->clearSolutionPaths();
        pdef_->addSolutionPath(solution);

        cout << "path cost: " << path->cost(opt_) << endl;
        cout << "path length: " << path->length() << endl;

        for(int i = 0; i < mpath.size() -1; i++)
        {
            auto s1 = mpath[i];
            auto s2 = mpath[i+1];
            if (!opt_->isCostBetterThan(getCost(s1,s2), opt_->infiniteCost()))
            {
                cout << "WARNING path has inf cost" << endl;
            }
        }
    }

    void RRTx::clear()
    {
        setup_ = false;
        Planner::clear();

        if(nn_->size() > 0)
        {
            vector<Motion *> motions;
            nn_->list(motions);
            for(auto m : motions)
            {
                if(m->state && m != goal_ && m != vbot_)
                    si_->freeState(m->state);
                delete m;
            }
        }

        // if(vbot_)
        // {
        //     delete vbot_;
        //     vbot_ = nullptr;
        // }

        iteration_ = 0;
        nn_->clear();
        q_.clear();
        motionHash_.clear();
        orphanHash.clear();

        if(pdef_)
        {
            pdef_->clearSolutionPaths();
        }
    }

    void RRTx::getPlannerData(PlannerData &data) const
    {
        Planner::getPlannerData(data);

        data.addGoalVertex(PlannerDataVertex(goal_->state));
        data.addStartVertex(PlannerDataVertex(vbot_->state));

        vector<Motion *> motions;
        nn_->list(motions);

        for(auto m : motions)
        {
            if(m->parent == nullptr)
                continue;
            data.addEdge(base::PlannerDataVertex(m->state), base::PlannerDataVertex(m->parent->state));
        }

        // for(auto m : motions)
        // {
        //     for(auto n : m->inNbhs())
        //     {
        //         if (opt_->isCostEquivalentTo(getCost(m, n), opt_->infiniteCost()))
        //             continue;
        //         data.addEdge(base::PlannerDataVertex(m->state), base::PlannerDataVertex(n->state));
        //     }
            
        // }

        cout << "vertices " << nn_->size() << endl;
        cout << "radius " << radius_ << endl;
        cout << "dimension " << si_->getStateDimension() << endl;
    }

    /*
    **  Algorithm: 1 grow() (part of algorithm 1)
    **  Add one Motion to the RRTx tree
    */
    void RRTx::grow()
    {
        //cout << "grow" << endl;
        updateRadius();

        Motion *v = sampleMotion();
        Motion *vnearest = nn_->nearest(v);
        saturate(v, vnearest);

        if(si_->isValid(v->state))
        {
            extend(v);
            if(v->parent != nullptr)
            {
                rewireNeighbors(v);
                reduceInconsistency();
            }

        }
        
        if(v->parent == nullptr && v != vbot_)
        {
            si_->freeState(v->state);
            delete v;
        }
    }



    /*  Algorithm 2: extend(Motion, radius_)
     *  
     *  Set parent and neighbors in radius to Motion
     */
    void RRTx::extend(Motion *v)
    {
        vector<Motion *> nbhs;
        nn_->nearestR(v, radius_ +0.001, nbhs);
        vector<Motion *> validMotions = findMotions(v, nbhs);

        // search for parent
        findParent(v, validMotions);
        if(v->parent == nullptr)
            return;

        // add vertex to the NearestNeighbor structure
        nn_->add(v);

        // set v 
        v->parent->children.push_back(v);

        //cout << "for nver : vnear" << endl;
        for(auto u : validMotions)
        {
            // Trajectory v -> u
            v->outN0.push_back(u);
            u->inNr.push_back(v);

            // Inverse trajectory u -> v.
            if  (symmetric_ ||
                (si_->distance(u->state, v->state) <= maxDist_ &&
                si_->checkMotion(u->state, v->state)))
            {
                costs_[std::make_pair(u, v)] = opt_->motionCost(u->state, v->state);
                u->outNr.push_back(v);
                v->inN0.push_back(u);
            }
            
        }
    }



    /*  
    **  Algorithm 3: cullNeighbors(v, r)
    **  reduce running neighbors.
    */
    void RRTx::cullNeighbors(Motion *v)
    {
        auto it = v->outNr.begin();

        while(it != v->outNr.end())
        {
            Motion *u = *it;
            if(v->parent != u && radius_ < distance(v, u))
            {
                it = v->outNr.erase(it);
                removeNbhs(u->inNr, v);
            }
            else
            {
                it++;
            }
        }
    }



    /*
    **  Algorithm 4: rewireNeighbors(Motion)
    **  Check if neighbors have better path through Motion
    */
    void RRTx::rewireNeighbors(Motion *v)
    {
        //  epsilon optimal
        auto newCost = opt_->combineCosts(epsilon, v->lmc);
        if(opt_->isCostBetterThan(newCost, v->g))
        {
            cullNeighbors(v);
            auto inNbhs = v->inNbhs();

            for(Motion *u : inNbhs)
            {
                if(u == v->parent)
                    continue;
                
                //  Find every neighbors with optimal path through v
                auto cost = opt_->combineCosts(getCost(u, v), v->lmc);
                if(opt_->isCostBetterThan(cost, u->lmc))
                {
                    u->lmc      = cost;
                    makeParentOf(v, u);

                    //  Check epsilon optimality for u
                    newCost = opt_->combineCosts(epsilon, u->lmc);
                    if(opt_->isCostBetterThan(newCost, u->g))
                        verrifyQueue(u);
                }
            }
        }
    }



    /*
    **  Algorithm: 5 reduceInconsistency()
    **  update lmc and rewire neighbors from Motions in the priority queue
    */
    void RRTx::reduceInconsistency()
    {
        while(  !q_.empty() &&
                (q_.top()->key < vbot_->key ||
                !opt_->isCostEquivalentTo(vbot_->lmc, vbot_->g) ||
                opt_->isCostEquivalentTo(vbot_->g, opt_->infiniteCost()) ||
                queueContains(vbot_)))
        {

            //Take first Motion from queue and remove it
            Motion *v = queuePop();
            
            auto cost = opt_->combineCosts(v->lmc, epsilon);
            if(opt_->isCostBetterThan(cost, v->g))
            {
                updateLMC(v);
                rewireNeighbors(v);
            }

            v->g = v->lmc;
        }
    }


    /*
    **  Algorithm 6: findParent(Motion, Neighbors)
    **  Find Parent for Motion v in list of neighbors
    */
    void RRTx::findParent(Motion *v, vector<Motion *> &nbh)
    {
        for(auto u : nbh)
        {
            Cost cost = opt_->combineCosts(getCost(v, u), u->lmc);
            // if path through u is better than before, make it parent
            if(opt_->isCostBetterThan(cost, v->lmc))
            {
                v->parent    = u;
                v->lmc       = cost;
            }
        }
        
    }



    /*
    **  Algorithm 12: verrifyQueue(Motion)
    **  Insert or Updates Motion in the priority queue
    */
    void RRTx::verrifyQueue(Motion *v)
    {
        if(queueContains(v))
            queueUpdate(v);
        else
            queueInsert(v);
    }

    

    /*
    **  Algorithm: 14 updateLMC(Motion)
    **  Updates parent of Motion if path through neighbor is better than the lmc
    */
    void RRTx::updateLMC(Motion *v)
    {
        cullNeighbors(v);
        Motion *p = v->parent;

        auto nbhs = v->outNbhs();
        for(Motion *u : nbhs)
        {
            if(orphanHash.find(u) != orphanHash.end())
                continue;

            if(u->parent == v)
                continue;

            Cost cost = opt_->combineCosts(getCost(v, u), u->lmc);
            if(opt_->isCostBetterThan(cost, v->lmc))
            {
                p = u;
            }
        }
        makeParentOf(p, v);
    }



    /*
    **  Distance function between two Motion
    */
    double RRTx::distance(Motion *v, Motion *u)
    {
        return si_->distance(v->state, u->state);
    }


    /*
    **  Returns a Motion v' between u and v with distance(u, v') < maxDist_
    */
    void RRTx::saturate(Motion *v, Motion *u)
    {
        if(v == vbot_) return;

        double dist = distance(v, u);
        if(dist > maxDist_)
        {
            si_->getStateSpace()->interpolate(u->state, v->state, maxDist_ / dist, v->state);
        }
    }


    /*
    **  Remove Motion in vector
    **  used by cullNeighbors(v, r)
    */
    void RRTx::removeNbhs(vector<Motion *> &nbhs, Motion *toRm)
    {
        for(auto it = nbhs.begin(); it != nbhs.end(); it++)
        {
            if(*it == toRm)
            {
                nbhs.erase(it);
                cout << "removeNbh true" << endl;
                return;
            }
        }
    }

    /*
    **  Get Cost between two Motion
    */
    Cost RRTx::getCost(Motion *a, Motion *b) const 
    {
        auto pair = std::make_pair(a, b);
        return costs_.find(pair)->second;
    }

    /**
     ** Find motions between v and neighbors that are obstacle free
     */
    vector<Motion *> RRTx::findMotions(Motion *v, vector<Motion *> motions)
    {
        vector<Motion *> exist;
        // add start Motion if v is near. vbot_ is not stored in the NearestNeigbhors datastructure

        for(auto u : motions)
        {
            if(si_->checkMotion(v->state, u->state))
            {
                costs_[std::make_pair(v, u)] = opt_->motionCost(v->state, u->state);
                exist.push_back(u);
            }
        }
        return exist;
    }

    /*
    **  Insert Motion in queue
    */
	void RRTx::queueInsert(Motion *v)
    {
        updateKey(v); 
        motionHash_[v] = q_.push(v);
    }
    
    /*
    **  Test if Queue contains Motion
    */
    bool RRTx::queueContains(Motion *v)
    {
        return motionHash_.find(v) != motionHash_.end();
    }
    
    /*
    **  Update Motion key and position in Queue
    */
    void RRTx::queueUpdate(Motion *v)
    {
        updateKey(v);
        q_.update(motionHash_[v]);
    }
    
    /*
    **  Remove Motion from Queue
    */
    void RRTx::queueRemove(Motion *v)
    {
        q_.erase(motionHash_[v]);
        motionHash_.erase(v);
    }

    /*
    ** Pop top Queue Motion
    */
    Motion *RRTx::queuePop()
    {
        Motion *toRemove = q_.top();
        motionHash_.erase(toRemove);
        q_.pop();
        return toRemove;
    }

    /*
    **  Set the key values
    */
    void RRTx::updateKey(Motion *v)
    {
        v->key.k1 = opt_->betterCost(v->g, v->lmc).value();
        v->key.k2 = v->g.value();
    }

    /*
    ** Update Radius based on number of Motions in the Tree
    */
    void RRTx::updateRadius()
    {
        // at start nn_ size is 1, so we add one to avoid 0 radius_
        int    n       = nn_->size() + 1;
        // double term1   = (y_ / M_PI) * (log(n) / n);
        // radius_  = min( pow(term1, 0.5), maxDist_);
        radius_ = y_ * std::pow(log(n) / n, 1.0 / si_->getStateDimension());
        // radius_ = y_ * std::pow(log(n) / n, 1 / 2.0);
        radius_ = min(radius_, maxDist_);
        //cout << "new radius_: " << radius_ << endl;
    }

    /* 
    **  returns a randomly sampled Motion 
    */
    Motion *RRTx::sampleMotion()
    {
        Motion *motion = new Motion;
        motion->state = si_->allocState();
        motion->lmc = opt_->infiniteCost();
        motion->g = opt_->infiniteCost();

        std::uniform_int_distribution<int> goal_uni(0, 10); // guaranteed unbiased
        auto random = goal_uni(rng);
        if(vbot_->parent == nullptr && random < 1)
        {
            return vbot_;
        }

        sampler_->sampleUniform(motion->state);

        return motion;
    }


    /*
    **  Set maxDistance between two neighbors
    */
    void RRTx::setRange(double dist)
    {
        maxDist_ = dist;
    }


    /*  
    *   set the parent of motion v and update children from old and new parent
    */ 
    void RRTx::makeParentOf(Motion *parent, Motion *v)
    {

        if(v->parent)
        {
            auto& children = v->parent->children;
            children.erase(remove_if(
                children.begin(), children.end(), [v](Motion *c){ return v == c; }),
                children.end());
        }

        v->parent = parent;
        if(v->parent)
        {
            parent->children.push_back(v);
        }
    }

    /**
     * Update Tree edges, and solution path
     */
    ob::PlannerStatus RRTx::updateTree(ob::State *center, double radius)
    {
        // ros::Time time = ros::Time::now();

        vector<Motion *> motions;
        Motion *m = new Motion();
        m->state = center;
        nn_->nearestR(m, radius + maxDist_, motions);
        cout << "update motions " << motions.size() << endl;
        //ROS_INFO("start update tree");
        // findFreeMotions(motions);
        // reduceInconsistency();

        findNewObstacles(motions);
        //ROS_INFO("end add obstacle");
        propogateDescendants();
        // vbot_->g = opt_->infiniteCost();
        verrifyQueue(vbot_);
        reduceInconsistency();
        //ROS_INFO("end update");

        // ros::Duration d = ros::Time::now() - time;
        //ROS_INFO("update Tree took %.3fsec", d.toSec());
        delete m;

        computePath();

        cout << "q " << q_.size() << endl;

        bool solved = !opt_->isCostEquivalentTo(vbot_->lmc, opt_->infiniteCost());
        return PlannerStatus(solved, false);

    }

    void RRTx::verrifyOrphan(Motion *v)
    {
        if(queueContains(v))
        {
            queueRemove(v);
        }

        orphanHash[v] = true;
    }

    void RRTx::insertOrphanChildren(Motion *v)
    {
        orphanHash[v] = true;

        for(auto c : v->children)
        {
            insertOrphanChildren(c);
        }
    }

    /**
     * Alogorithm 8: PropogateDescendants(
     * */
    void RRTx::propogateDescendants()
    {
        //ROS_INFO("propagateDescendants");
        // get all orphan motions
        vector<Motion *> orphans;
        orphans.reserve(orphanHash.size());
        for(auto o : orphanHash) 
        { 
            orphans.push_back(o.first);
        }
        cout << "orphans " << orphans.size() <<endl;
        // insert children to orphans
        for(auto v : orphans)
        {
            insertOrphanChildren(v);
        }
        // get new list of orphan motions
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
            vector<Motion *> nbhs = v->outNbhs();
            nbhs.push_back(v->parent);
            for(auto n : nbhs)
            {
                if(orphanHash.find(n) != orphanHash.end()) continue;

                n->g = opt_->infiniteCost();
                verrifyQueue(n);
            }

        }

        for (auto v : orphans)
        {
            v->g = opt_->infiniteCost();
            v->lmc = opt_->infiniteCost();
            if (v->parent)
            {
                auto &children = v->parent->children;
                children.erase(remove_if(
                    children.begin(), children.end(), [v](Motion *c) { return v == c; }),
                    children.end());
                v->parent = nullptr;
            }
        }

        orphanHash.clear();
    //  ROS_INFO("end propagate");
    }

    /**
     * Algorithm 11: FindNewObstacles
     * Search for Edges with obstacles and set the cost to inf
     */
    void RRTx::findNewObstacles(vector<Motion *> &motions)
    {   
        int counter = 0;
        for(auto v : motions)
        {
            auto nbhs = v->outNbhs();
            for(auto u : nbhs)
            {
                Cost c = getCost(v, u);
                if(opt_->isCostEquivalentTo(c, opt_->infiniteCost()) && orphanHash.find(v) != orphanHash.end()) 
                    continue;

                if(!si_->checkMotion(v->state, u->state))
                {
                    counter++;
                    costs_[make_pair(v, u)] = opt_->infiniteCost();
                    costs_[make_pair(u, v)] = opt_->infiniteCost();

                    if(v->parent == u)
                    {
                        verrifyOrphan(v);
                    }
                }
            }
        }

        cout << "edge collision " << counter << endl;
    }

    /**
     * FindFreeMotions(motions)
     * Finds motion that are free and updates their cost
     * */
    void RRTx::findFreeMotions(std::vector<Motion *> &motions)
    {
        for(auto v : motions)
        {
            bool changed = false;
            auto nbhs = v->outNbhs();
            for(auto u : nbhs)
            {
                if(!opt_->isCostEquivalentTo(getCost(v, u), opt_->infiniteCost())) 
                    continue;

                if(si_->checkMotion(v->state, u->state))
                {
                    changed = true;
                    costs_[make_pair(v, u)] = opt_->motionCost(v->state, u->state);
                }
            }
            if(changed)
                updateLMC(v);
            
            if(!opt_->isCostEquivalentTo(v->lmc, v->g))
                verrifyQueue(v);
        }
    }

    // void RRTx::updateRobot(geometry_msgs::Pose robot)
    // {
    //     // Motion vbot;
    //     // vbot.x = robot.position.x;
    //     // vbot.y = robot.position.y;

    //     // if(lastPath.size() > 3)
    //     // {
    //     //     if(distance(vbot, *lastPath[1]) > distance(vbot, *lastPath[2]) && getCost(lastPath[1], lastPath[2]) != infinity)
    //     //     {
    //     //         vbot_ = lastPath[1];
    //     //         makeParentOf(lastPath[2], vbot_);
    //     //         updateKey(vbot_);
    //     //     }
    //     // }
    // }


}; // namespace rrt