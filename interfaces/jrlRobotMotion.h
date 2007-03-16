#ifndef JRL_ROBOT_MOTION_H
#define JRL_ROBOT_MOTION_H

#include "robotDynamics/jrlHumanoidDynamicRobot.h"

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
    virtual ~CjrlRobotMotion() {}

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
    virtual const CjrlHumanoidDynamicRobot& robot() = 0;

    /**
    \brief Get Configuration at given time.
     */
    virtual bool configAtTime(double inTime, vectorN& outConfig) = 0;

    /**
    \brief Get velocity at given time.
     */
    virtual bool velocityAtTime(double inTime, vectorN& outVector) = 0;

    /**
    \brief Get Acceleration at given time.
     */
    virtual bool accelerationAtTime(double inTime, vectorN& outVector) = 0;

};

#endif
