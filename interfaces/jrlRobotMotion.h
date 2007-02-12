#ifndef JRL_ROBOT_MOTION_H
#define JRL_ROBOT_MOTION_H

#include "jrlDynamicRobot.h"

/**
   \brief Defines the motion of a robot along time.
 
   The motion is defined over an interval of time.
 */


template <class Mnxp,class M4x4,class M3x3,class Vn,class V3>
class CjrlRobotMotion
{
public:
    /**
    \brief Destructor.
     */
    virtual ~CjrlRobotMotion() =0;

    /**
    \brief Get lower bound of definition interval.
     */
    virtual double startTime() = 0;

    /**
    \brief Get upper bound of defintion interval.
     */
    virtual double endTime() = 0;

    /**
    \brief Get robot for which motion is defined.
     */
    virtual const CjrlDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>& robot() = 0;

    /**
    \brief Get Configuration at given time.

    \return The configuration vector.
     */
    virtual Vn configAtTime(double inTime) = 0;

    /**
    \brief Get velocity at given time.
     */
    virtual V3 velocityAtTime(double inTime) = 0;

    /**
    \brief Get Acceleration at given time.
     */
    virtual V3 accelerationAtTime(double inTime) = 0;
    
    /**
    \brief Append a new configuration at the end of the current motion (must be in the implementation )
     */
    virtual void appendConfig(const Vn&) = 0;
    
    /**
    \brief Clear stored motion
     */
    virtual void clear() = 0;
};

#endif
