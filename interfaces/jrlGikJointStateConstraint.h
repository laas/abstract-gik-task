#ifndef JRL_JOINT_STATE_CONSTRAINT_H
#define JRL_JOINT_STATE_CONSTRAINT_H

#include "jrlGikStateConstraint.h"

/**
\brief Specify a Constraint over the state of a joint in a humanoid robot.
 */


class CjrlGikJointStateConstraint: public CjrlGikStateConstraint
{
public:
    /**
        \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint) =0;

    /**
        \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint() =0;
};


#endif
