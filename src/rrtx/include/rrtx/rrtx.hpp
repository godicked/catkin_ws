#ifndef RRTX_HPP
#define RRTX_HPP

#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>
#include <rrtx/rrtx_struct.hpp>

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

    public:

            RRTx(ob::SpaceInformationPtr si);
                                
            void setMaxDist(double maxDist);

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

            // void                updateTree      (double origin_x, double origin_y, double size_x, double size_y);
            
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

            void reduceIncosistency(); //

            void updateLMC(Motion *v); //

            void updateRadius(); //

            void makeParentOf(Motion *parent, Motion *v); //

            Motion *sampleMotion(); //

            void grow(); //

            ob::Cost getCost(Motion *a, Motion *b); //

            //  Priority Queue related functions
            void queueInsert(Motion *v); //

            void queueUpdate(Motion *v); //

            void queueRemove(Motion *v); //

            bool queueContains(Motion *v); //

            void updateKey(Motion *v); //

            Motion *queuePop(); //

            // Dynamic part of RRTx
            // void addObstacle(double origin_x, double origin_y, double size_x, double size_y);
            // void findFreeTrajectories(costmap_2d::Costmap2D map);
            // void propogateDescendants();
            // void verrifyOrphan(Motion *v);
            // void insertOrphanChildren(Motion *v);

            // OMPL NearestNeigbhors interface for KNN and distance search
            std::shared_ptr<ompl::NearestNeighbors<Motion *> > nn_;

            // Orphan Nodes after adding obstacles
            OrphanHash orphanHash;

            //  The priority queue needed by the RRTx algorithm
            //  And a NodeHash table to save the handle of inserted Motions
            //  Allows contains/updade/remove operations
            Queue q_;
            MotionHash motionHash_;

            //  max distance between 2 a new point and its nearest neighbor
            double maxDist_ = 4.55;


            ob::Cost epsilon = ob::Cost(0.05);

            double  radius_;
            double  y_;
            unsigned int iteration_;

            Motion    *vbot_ = nullptr;
            Motion    *goal_ = nullptr;

            ob::OptimizationObjectivePtr opt_;
            ob::StateSamplerPtr sampler_;

            int fail = 0;
            std::vector<ob::State *> failed;

    };

};

#endif
