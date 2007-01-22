#ifndef JRL_GIK_WHOLE_BODY_MOTION_PLANNER_H
#define JRL_GIK_WHOLE_BODY_MOTION_PLANNER_H

#include "jrlGikStateConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlGikWholeBodyMotionTask.h"
#include "jrlRobotMotion"
#include <boost/numeric/ublas/matrix.hpp>

/**
   \brief A Whole body motion planner takes in ready motion constraints or desired state constraints to define a whole body motion task and solve it. 
 */

//temporary
typedef ublas::matrix<double> CjrlRobotMotion;

class CjrlGikWholeBodyMotionPlanner
{
public:
    /**
    \name Definition
    @{
     */

    /**
    \brief Add a motion constraint described by a desired state constraint. The state constraint is transformed by the implemented algorithms into a set of motion constraints and added to the whole body motion task.
     */
    virtual addStateConstraint(const CjrlGikStateConstraint& inStateConstraint) = 0;

    /**
    \brief Add a ready motion constraint. The motion constraint is added as-is to the whole body motion task.
     */
    virtual addMotionConstraint(const CjrlGikMotionConstraint& inMotionConstraint) = 0;

    /**
    \brief Get the so-far-defined whole body motion task.
     */
    virtual CjrlGikWholeBodyMotionTask* wholeBodyMotionTask() = 0;

    /**
    @}
     */

    /**
    \name Resolution
    @{
     */

    /**
    \brief Compute a motion complying with the constraints contained in whole body motion task.
     */
    virtual bool solve() = 0;

    /**
    \brief Get resulting motion of robot.
     */
    virtual const CjrlRobotMotion* solutionMotion() = 0;
    /**
    @}
     */
};

#endif
