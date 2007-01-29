#ifndef JRL_GIK_MOTION_CONSTRAINT_H
#define JRL_GIK_MOTION_CONSTRAINT_H


#include "jrlGik/jrlGikStateConstraint.h"

/**
\brief Define the evolution of a state constraint along time
 */

class CjrlGikMotionConstraint
{
public:

    virtual CjrlGikMotionConstraint(double inSamplingPeriod);
            
    /**
    \brief Append a state constraint at the end of the motion.
     */
    virtual void appendStateConstraint(const CjrlGikStateConstraint& inStateConstraint) = 0;

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
    /**
    \brief Destructor
     */
    virtual ~CjrlGikMotionConstraint() =0;

};


#endif
