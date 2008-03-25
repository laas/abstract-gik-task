#ifndef JRL_GIK_MOTION_CONSTRAINT_H
#define JRL_GIK_MOTION_CONSTRAINT_H


#include "jrlGikStateConstraint.h"

/**
\brief Define the evolution of a state constraint along time
 */

class CjrlGikMotionConstraint
{
public:
    /**
    \brief Get a pointer to associated robot
     */
    virtual CjrlDynamicRobot* robot() = 0;
    
    /**
    \brief Clone constructor
    */
    virtual CjrlGikMotionConstraint* clone() const =0;
    
    /**
    \brief Get state constraint at a given time.
     */
    virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime) = 0;

    /**
    \brief Set lower bound of definition interval.
     */
    virtual void startTime(double inStartTime)=0;

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
    virtual ~CjrlGikMotionConstraint()
    {}

};

#endif
