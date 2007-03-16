#ifndef JRL_GIK_POINTING_CONSTRAINT_H
#define JRL_GIK_POINTING_CONSTRAINT_H

#include "jrlGikJointStateConstraint.h"

/**
\brief Constraint on a line segment attached to a body to be aligned with a given point in the world frame. The line segment is defined by an origin point and a vector both given in the body's local frame.
 */

class CjrlGikPointingConstraint:public CjrlGikJointStateConstraint
{
public:

    /**
    \brief Set the origin of the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
    virtual void  localOrigin(const vector3d& inPoint) = 0;
    /**
    \brief Get the origin of the pointing vector.
     */
    virtual const vector3d& localOrigin() = 0;
    /**
    \brief Set the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
    virtual void  localVector(const vector3d& inPoint) = 0;
    /**
    \brief Set the pointing vector in joint's local frame
     */
    virtual const vector3d& localVector() = 0;
    /**
    \brief Set the target point associated to the constraint (in world's frame).
     */
    virtual void  worldTarget(const vector3d& inPoint) = 0;
    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const vector3d& worldTarget() = 0;

};

#endif
