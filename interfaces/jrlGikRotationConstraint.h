/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_ROTATION_CONSTRAINT_H
#define JRL_GIK_ROTATION_CONSTRAINT_H

#include "jrlGikJointStateConstraint.h"

/**
\brief Constraint on the orientation matrix of a joint to change to a given orientation matrix
 */
class CjrlGikRotationConstraint:virtual public CjrlGikJointStateConstraint
{
public:

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
