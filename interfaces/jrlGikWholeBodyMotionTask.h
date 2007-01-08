#ifndef JRL_GIK_WHOLE_BODY_MOTION_TASK_H
#define JRL_GIK_WHOLE_BODY_MOTION_TASK_H

#include "jrlRobotMotion"

/**
   \brief A Whole body motion task is a set of prioritized motion constraints
*/
class CjrlGikWholeBodyMotionTask {
public:
  /**
     \name Definition
     @{
  */
  /**
     \brief Add a Motion constraint.
  */
  virtual addMotionConstraint(const CjrlGikMotionConstraint& inMotionConstraint) = 0;

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
