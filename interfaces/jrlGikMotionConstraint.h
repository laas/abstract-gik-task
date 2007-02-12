#ifndef JRL_GIK_MOTION_CONSTRAINT_H
#define JRL_GIK_MOTION_CONSTRAINT_H


#include "jrlGikStateConstraint.h"

/**
\brief Define the evolution of a state constraint along time
 */
template <class Mnxp,class M4x4,class M3x3,class Vn,class V3>
class CjrlGikMotionConstraint
{
public:

    /**
    \brief Get state constraint at a given time.
     */
    virtual CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>* stateConstraintAtTime(double inTime) = 0;

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
    virtual ~CjrlGikMotionConstraint() =0;

};


#endif
