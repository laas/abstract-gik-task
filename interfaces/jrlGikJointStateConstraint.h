#ifndef JRL_JOINT_STATE_CONSTRAINT_H
#define JRL_JOINT_STATE_CONSTRAINT_H

#include "jrlGikStateConstraint.h"

/**
\brief Specify a Constraint over the state of a joint in a humanoid robot.
 */

template <class Mnxp,class M4x4,class M3x3,class Vn,class V3>
class CjrlGikJointStateConstraint:public CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>
{
public:
    /**
        @}
     */
    /**
        \brief Destructor
     */
    virtual ~CjrlGikJointStateConstraint() = 0;

    /**
    \brief Copy
     */
    virtual CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>* clone() const =0;

    /**
        \name Definition of the constraint
        @{
     */

    /**
        \brief Get associated robot
     */
    virtual CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>& robot() = 0;

    /**
        \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inJoint) =0;

    /**
        \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* joint() =0;

    /**
        \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const = 0;

    /**
        @}
     */

    /**
        \name Computations
        @{
     */

    /**
        \brief Compute the value of the constraint.
     */
    virtual void computeValue() = 0;

    /**
        \brief Compute the Jacobian matrix of the constraint value wrt all (internal and external) configuration variables.
        The contacts with the world are not taken into account
     */
    virtual void computeJacobianFromRoot() = 0;

    /**
        \brief Compute the Jacobian matrix of the constraint value wrt internal configuration variables.
        The interaction with the environment is taken into account (for instance a foot on the ground).
     */
    virtual void computeJacobian() = 0;

    /**
        @}
     */

    /**
        \name Getting result of computations
        @{
     */

    /**
        \brief Get the constraint value.
     */
    virtual const Vn& value() = 0;

    /**
        \brief Get the constraint Jacobian wrt all (internal and external) configuration variables.
        The contacts with the world are not taken into account
     */
    virtual const Mnxp& jacobianFromRoot() = 0;

    /**
        \brief Get the constraint Jacobian wrt internal configuration variables.
        The interaction with the environment is taken into account (for instance a foot on the ground)
     */
    virtual const Mnxp& jacobian() = 0;

    /**
        @}
     */

};



#endif
