#ifndef JRL_GIK_WHOLE_BODY_MOTION_TASK_H
#define JRL_GIK_WHOLE_BODY_MOTION_TASK_H

#include "jrlRobotMotion"

/**
   \brief A Whole body motion task is a set of prioritized motion constraints. 

   The priority is defined by the rank in the heap of constraints. The lower the rank, the higher the priority.
*/

class CjrlGikWholeBodyMotionTask {
public:
  /**
     \name Definition
     @{
  */
  /**
     \brief Add a motion constraint in heap.
  */
  virtual addMotionConstraint(const CjrlGikMotionConstraint& inMotionConstraint) = 0;

  /**
     \brief Remove a motion constraint from the heap.

     \param inRank rank of the constraint to remove in the heap.
     \return TRUE if success, FALSE if rank is more than number of constraints.
  */
  virtual bool removeMotionConstraint(unsigned int inRank) = 0;

  /**
     \brief Get number of motion constraints.
  */
  virtual unsigned int numberMotionConstraints() = 0;

  /**
     \brief Get Motion constraint at given rank.
  */
  virtual const CjrlGikMotionConstraint* motionConstraint(unsigned int inRank) = 0;

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
