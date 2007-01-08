#ifndef JRL_GIK_MOTION_CONSTRAINT_H
#define JRL_GIK_MOTION_CONSTRAINT_H

/**
   \brief Define the evolution of a state constraint along time
*/

class CjrlGikMotionConstraint {
public:
  /**
     \brief Constructor 
     \param inStartTime Lower bound of the definition interval of the motion constraint.
     \param inEndTime Upper bound of the definition interval of the motion constraint.
  */
  CjrlGikMotionConstraint(double inStartTime, double inEndTime);

  /**
     \brief Get state constraint at a given time.
  */
  virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime) = 0;

  /**
     \brief Get lower bound of definition interval.
  */
  virtual double startTime() = 0;

  /**
     \brief Get upper bound of definition interval.
  */
  virtual double endTime() = 0;

};


#endif
