/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_TRANSFORMATION_CONSTRAINT_H
#define JRL_GIK_TRANSFORMATION_CONSTRAINT_H

#include "jrlGikJointStateConstraint.h"

/**
\brief Specify a position and orientation constraint on a body of the robot.
 */

class CjrlGikTransformationConstraint:virtual public CjrlGikJointStateConstraint
{
public:
  
    /**
    \brief Set the point (in joint's local frame) associated to the constraint.
     */
    virtual void  localPoint(const vector3d& inPoint) = 0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const vector3d& localPoint() = 0;
    /**
    \brief Set the target point associated to the constraint (in world's frame).
     */
    virtual void  worldTarget(const vector3d& inPoint) = 0;
    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const vector3d& worldTarget() = 0;
    /**
    \brief Set the target orientation for this constraint.
     */
    virtual void  targetOrientation(const matrix3d& inRot)=0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const matrix3d& targetOrientation()=0;
};

#endif
