#ifndef RRTX_HPP
#define RRTX_HPP

#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>
#include <rrtx/RRTxStruct.hpp>
#include <ompl/datastructures/NearestNeighborsGNAT.h> 
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/State.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/datastructures/BinaryHeap.h>


// namespace ob = ompl::base;

using namespace boost;

namespace rrt
{

    class RRTx : public ob::Planner
    {

        // typedef boost::heap::fibonacci_heap<Motion *, 
        //         boost::heap::compare<motion_compare> > Queue;
        typedef ompl::BinaryHeap<Motion *, motion_compare > Queue;
        typedef Queue::Element* handle_t;
        typedef boost::unordered_map<Motion *, handle_t> MotionHash;
        typedef boost::unordered_map<Motion *, bool> OrphanHash;
        typedef boost::unordered_map<MotionPair, ob::Cost, hash_motion_pair, motion_pair_equal> MotionCosts;

    public:

            RRTx(ob::SpaceInformationPtr si);
            
            //  Maximum distance between two sampled states
            void setRange(double maxDist);

            //  Set epsilon
            void setEpsilon(double e)
            {
                epsilon = ob::Cost(e);
            }

            void setGoalBias(int bias)
            {
                goal_bias_ = bias;
            }

            //  Default solve with time as termination condition from base class
            using Planner::solve;

            //  RRTx solve function
            virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

            //  Setup called for structure initialisation
            virtual void setup() override;

            //  Clear the Tree for new planning
            virtual void clear() override;

            //  Get the optimal Tree
            virtual void getPlannerData(ob::PlannerData &data) const override;

            //  Actual iterations
            unsigned int numIterations()
            {
                return iteration_;
            }

            //  Vertices in the Tree
            unsigned int numVertices()
            {
                return nn_->size();
            }

            ob::PlannerStatus updateTree(ob::State *center, double radius); //
            
            ob::PlannerStatus updateRobot(ob::State *robot);

            ob::PlannerStatus verifyPath();

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
            void queueInsert(Motion *v); 

            void queueUpdate(Motion *v); 

            void queueRemove(Motion *v);

            bool queueContains(Motion *v);

            void updateKey(Motion *v);

            Motion *queuePop(); 

            // Dynamic part of RRTx

            void findNewObstacles(std::vector<Motion *> &motions);

            void findFreeMotions(std::vector<Motion *> &motions);

            void propogateDescendants();

            void verrifyOrphan(Motion *v); 

            void insertOrphanChildren(Motion *v);

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
            unsigned int goal_bias_ = 10; // 1 every 50 samples
            bool force_goal_bias_ = false; // used to sample the new robot position

            Motion    *vbot_ = nullptr;
            Motion    *goal_ = nullptr;

            ob::OptimizationObjectivePtr opt_;
            ob::StateSamplerPtr sampler_;

            std::vector<ompl::base::State *> path_;
    };

};

#endif
