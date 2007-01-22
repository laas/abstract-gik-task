#ifndef JRL_GIK_WHOLE_BODY_MOTION_TASK_H
#define JRL_GIK_WHOLE_BODY_MOTION_TASK_H

#include "jrlGikMotionConstraint.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

/**
   \brief A Whole body motion task is a set of prioritized motion constraints.
*/

//temporary ?
typedef ublas::vector<double> CjrlRobotConfiguration ;
//temporary ?
typedef ublas::matrix<double> CjrlRobotMotion;

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
  \brief Compute a motion complying with the constraints contained in whole body motion task at time inTimeTick.
   */
  virtual bool solveForTime(unsigned int inTimeTick) = 0;
  /**
  \brief Get solution configuration 
   */
  virtual CjrlRobotConfiguration& solutionConfig() = 0;

  /**
  \brief Solve entire motion (when no dynamic planning is expected for example).
   */
  virtual bool solve(unsigned int inTimeTick) = 0;
    /**
  \brief Get solution motion
     */
  virtual CjrlRobotMotion& solutionMotion() = 0;
  
  
  


  /**
  @}
   */
};

#endif
