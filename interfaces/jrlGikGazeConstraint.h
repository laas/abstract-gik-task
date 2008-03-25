#ifndef JRL_GIK_GAZE_CONSTRAINT_H
#define JRL_GIK_GAZE_CONSTRAINT_H

#include "jrlGikPointingConstraint.h"

/**
\brief This is a PointingConstraint whose joint is the humanoid robot's gaze joint. Only the target point in the world is provided.
 */


class CjrlGikGazeConstraint:virtual public CjrlGikPointingConstraint
{
public:

    /**
        \brief Set the joint associated to the constraint (should be an illelgal operation for gaze constraint).
     */
    virtual void  joint(CjrlJoint* inJoint)
    {}
    /**
    \brief Set the origin of the pointing vector in joint's local frame (should be an illelgal operation for gaze constraint).
     */
    virtual void  localOrigin(const vector3d& inPoint)
    {}
    /**
    \brief Set the pointing vector in joint's local frame (should be an illelgal operation for gaze constraint).
     */
    virtual void  localVector(const vector3d& inPoint)
    {}

};

#endif
