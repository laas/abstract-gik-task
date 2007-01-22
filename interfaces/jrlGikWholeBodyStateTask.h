#ifndef JRL_GIK_WHOLE_BODY_STATE_TASK_H
#define JRL_GIK_WHOLE_BODY_STATE_TASK_H

#include "jrlGik/jrlGikWholeBodyStateSubtask.h"
#include "jrlGik/jrlGikTools.h"
#include "jrlRobot/jrlHumanoidDynamicRobot.h"

#include <boost/numeric/ublas/matrix.hpp>

namespace ublas = boost::numeric::ublas;
using namespace std;

/**
\brief A Whole body state task is a set of prioritized subtasks. The priority of the subtask determines its rank in the vector of pointers. The first element in the vector of subtasks has the highest priority (lowest priority number).
 */
class CjrlGikWholeBodyStateTask
{
    public:
    /**
        \name Definition
        @{
     */

    /**
        \brief Add a subtask. The task is automatically inserted at the adequate rank according to its priority.
     */
        virtual void addSubtask(CjrlGikWholeBodyStateSubtask* inStateSubtask)=0;
    /**
        \brief Insert a subtask at a given rank. Assumes the user is observing the priority order.
     */
        virtual bool insertSubtask(CjrlGikWholeBodyStateSubtask* inStateSubtask, unsigned int inRank)=0;
    
    /**
        \brief Get the number of subtasks.
     */
        virtual unsigned int numberSubtasks()=0;

    /**
        \brief Get the subtask at the given rank.
     */
        virtual const CjrlGikWholeBodyStateSubtask* stateSubtask(unsigned int inRank)=0;

    /**
        @}
     */

    /**
        \name Resolution
        @{
     */

    /**
        \brief Compute a configuration complying with the whole body state task.
     */
        virtual bool solve()=0;

    /**
        \brief Get the computed configuration of the robot.
     */
        virtual const ublas::vector<double>& solutionConfiguration()=0;
    /**
        @}
     */
        virtual ~CjrlGikWholeBodyStateTask() =0;

};

#endif
