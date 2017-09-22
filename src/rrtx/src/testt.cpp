#include <rrtx/reeds_shepp_config.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/MotionValidator.h>

#include <rrtx/rrtx.hpp>
// #include <ompl/base/spaces/ReedsSheppStateSpace.h>
// #include <ompl/base/SpaceInformation.h>

using namespace ompl::geometric;
using namespace ompl::base;
using namespace costmap_2d;
using namespace rrt;
using namespace std;

int main(int argc, char **argv)
{

    Costmap2D *cm;

    // ReedsSheppStateSpace ss;
    StateSpacePtr ss( new CostmapStateSpace(cm) );
    SpaceInformationPtr si( new SpaceInformation(ss));
    StateValidityCheckerPtr svcp( new CostmapValidityChecker(si.get(), cm) );
    OptimizationObjectivePtr oop( new CostmapOptimizationObjective(si, cm) );
    ProblemDefinitionPtr pdp( new ProblemDefinition(si) );
    // MotionValidatorPtr mvp( new ReedsSheppMotionValidator(si.get()));

    pdp->setOptimizationObjective( oop );
    si->setStateValidityChecker( svcp );

    
    RRTx planner(si);
    planner.init(pdp);
}