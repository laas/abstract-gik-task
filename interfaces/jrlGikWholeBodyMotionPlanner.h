#ifndef JRL_GIK_WHOLE_BODY_MOTION_PLANNER_H
#define JRL_GIK_WHOLE_BODY_MOTION_PLANNER_H

#include "jrlGikStateConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlRobotMotion.h"

/**
\brief A Whole body motion planner takes in state or motion constraints to define a whole body motion task and solve it.
 */

template <class Mnxp,class M4x4,class M3x3,class Vn,class V3>
class CjrlGikWholeBodyMotionPlanner
{
    public:
    /**
        \name Definition
        @{
     */
    /**
        \brief Get associated robot
     */
        virtual CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>& robot()const =0 ;

    /**
        \brief Resets the planner by clearing entered constraints
     */
        virtual void reset() = 0;
    
    /**
        \brief Add a prioritized state constraint. The state constraint is transformed by the implemented algorithms into motion subtasks which are added to the whole body motion task.
     */
        virtual void addStateConstraint(CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>* inStateConstraint, unsigned int inPriority) = 0;

    /**
        \brief Add a prioritized motion constraint. The motion constraint is inserted as-is to the whole body motion task. Simultaneous motion subtasks might be generated by the implemented algorithms.
     */
        virtual void addMotionConstraint(CjrlGikMotionConstraint<Mnxp,M4x4,M3x3,Vn,V3>* inMotionConstraint, unsigned int inPriority) = 0;
    /**
        @}
     */

    /**
        \name Resolution
        @{
     */
    
    /**
        \brief Plan a motion to realize the given configuration.
        The solution retrieved through solutionMotion()
     */
        virtual bool goToConfiguration(const Vn& inRobotConfiguration) =0;
        
    /**
        \brief Compute a motion to realize the constraints contained in whole body motion task.
        The solution retrieved through solutionMotion()
     */
        virtual bool solve() = 0;

    /**
        \brief Get resulting joint motion of robot.
     */
        virtual const CjrlRobotMotion<Mnxp,M4x4,M3x3,Vn,V3>& solutionMotion() = 0;
    /**
        @}
     */
    /**
        \brief Destructor
     */
        virtual ~CjrlGikWholeBodyMotionPlanner() =0;
    
};

#endif
