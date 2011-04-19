#ifndef JRL_ROBOT_MOTION_H
#define JRL_ROBOT_MOTION_H

#include "abstract-robot-dynamics/humanoid-dynamic-robot.hh"

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
    virtual double startTime() const = 0 ;

    /**
    \brief Get upper bound of defintion interval.
     */
    virtual double endTime() const = 0;

    /**
    \brief Get robot for which motion is defined.
     */
    virtual const CjrlHumanoidDynamicRobot& robot() = 0;

    /**
    \brief Get Configuration at given time.
     */
    virtual bool configAtTime(double inTime, vectorN& outConfig) const = 0;

    /**
    \brief Get velocity at given time.
     */
    virtual bool velocityAtTime(double inTime, vectorN& outVector) const = 0;

    /**
    \brief Get Acceleration at given time.
     */
    virtual bool accelerationAtTime(double inTime, vectorN& outVector) const = 0;

};

#endif
