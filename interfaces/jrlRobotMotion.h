#ifndef JRL_ROBOT_MOTION_H
#define JRL_ROBOT_MOTION_H

#include "jrlDynamicRobot.h"


namespace ublas = boost::numeric::ublas;
typedef  ublas::vector<double> uvector;
/**
   \brief Defines the motion of a robot along time.
 
   The motion is defined over an interval of time.
 */


class CjrlRobotMotion
{
public:
    /**
    \brief Destructor.
     */
    virtual ~CjrlRobotMotion();

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
    virtual const CjrlDynamicRobot& robot() = 0;

    /**
    \brief Get Configuration at given time.

    \return The configuration vector.
     */
    virtual uvector configAtTime(double inTime) = 0;

    /**
    \brief Get velocity at given time.
     */
    virtual uvector velocityAtTime(double inTime) = 0;

    /**
    \brief Get Acceleration at given time.
     */
    virtual uvector accelerationAtTime(double inTime) = 0;
    
    /**
    \brief Append a new configuration at the end of the current motion (must be in the implementation )
     */
    virtual void appendConfig(const uvector&) = 0;
    
    /**
    \brief Clear stored motion
     */
    virtual void clear() = 0;
};

#endif
