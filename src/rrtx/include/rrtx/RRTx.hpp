#ifndef RRTX_HPP
#define RRTX_HPP

#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>
#include <rrtx/RRTxStruct.hpp>

// #include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h> 
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/State.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>


namespace ob = ompl::base;

using namespace boost;

namespace rrt
{

    class RRTx : public ob::Planner
    {

        typedef boost::heap::fibonacci_heap<Motion *, 
                boost::heap::compare<motion_compare> > Queue;
        typedef Queue::handle_type handle_t;
        typedef boost::unordered_map<Motion *, handle_t> MotionHash;
        typedef boost::unordered_map<Motion *, bool> OrphanHash;
        typedef boost::unordered_map<MotionPair, ob::Cost, hash_motion_pair, motion_pair_equal> MotionCosts;

    public:

            RRTx(ob::SpaceInformationPtr si);
                                
            void setRange(double maxDist);

            using Planner::solve;
            virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

            virtual void setup() override;

            virtual void clear() override;

            virtual void getPlannerData(ob::PlannerData &data) const override;

            unsigned int numIterations()
            {
                return iteration_;
            }

            unsigned int numVertices()
            {
                return nn_->size();
            }

            void freeMemory();

            void setSearchPath(std::vector<ompl::base::State *> path, double sample_dist)
            {
                path_ = path;
                path_sample_ = true;
                sample_dist_ = sample_dist;
            }

            ob::PlannerStatus updateTree(ob::State *center, double radius); //
            
            // void                updateRobot (geometry_msgs::Pose robot);

    private:

            double distance(Motion *v, Motion *u);//

            void saturate(Motion *v, Motion *u); //

            void findParent(Motion *v, std::vector<Motion *> &nbh); //

            std::vector<Motion *> findMotions(Motion *v, std::vector<Motion *> nodes); //

            void removeNbhs(std::vector<Motion *> &nbhs, Motion *toRm); //

            void extend(Motion *v); //

            void cullNeighbors(Motion *v); //

            void verrifyQueue(Motion *v); //

            void rewireNeighbors(Motion *v); //

            void reduceInconsistency(); //

            void updateLMC(Motion *v); //

            void updateRadius(); //

            void makeParentOf(Motion *parent, Motion *v); //

            Motion *sampleMotion(); //

            void grow(); //

            ob::Cost getCost(Motion *a, Motion *b) const; //

            //  Priority Queue related functions
            void queueInsert(Motion *v); //

            void queueUpdate(Motion *v); //

            void queueRemove(Motion *v); //

            bool queueContains(Motion *v); //

            void updateKey(Motion *v); //

            Motion *queuePop(); //

            // Dynamic part of RRTx

            void findNewObstacles(std::vector<Motion *> &motions); //

            void findFreeMotions(std::vector<Motion *> &motions); //

            void propogateDescendants(); //

            void verrifyOrphan(Motion *v); //

            void insertOrphanChildren(Motion *v); //

            void computePath();

            // OMPL NearestNeigbhors interface for KNN and distance search
            std::shared_ptr<ompl::NearestNeighbors<Motion *> > nn_;

            // Orphan Nodes after adding obstacles
            OrphanHash orphanHash;

            //  The priority queue needed by the RRTx algorithm
            //  And a NodeHash table to save the handle of inserted Motions
            //  Allows contains/updade/remove operations
            Queue q_;

            MotionHash motionHash_;
            MotionCosts costs_;

            //  max distance between 2 a new point and its nearest neighbor
            double maxDist_ = 4.55;
            bool symmetric_;


            ob::Cost epsilon = ob::Cost(0.05);

            double  radius_;
            double  y_;
            unsigned int iteration_;

            Motion    *vbot_ = nullptr;
            Motion    *goal_ = nullptr;

            ob::OptimizationObjectivePtr opt_;
            ob::StateSamplerPtr sampler_;

            std::vector<ompl::base::State *> path_;
            bool path_sample_ = false;
            double sample_dist_ = 0;

    };

};

#endif
