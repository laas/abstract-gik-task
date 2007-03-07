#ifndef JRL_ROBOT_MOTION_H
#define JRL_ROBOT_MOTION_H

#include "jrlHumanoidDynamicRobot.h"

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
    virtual const CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>& robot() = 0;

    /**
    \brief Get Configuration at given time.
     */
    virtual bool configAtTime(double inTime, Vn& outConfig) = 0;

    /**
    \brief Get velocity at given time.
     */
    virtual bool velocityAtTime(double inTime, Vn& outVector) = 0;

    /**
    \brief Get Acceleration at given time.
     */
    virtual bool accelerationAtTime(double inTime, Vn& outVector) = 0;

};

#endif
