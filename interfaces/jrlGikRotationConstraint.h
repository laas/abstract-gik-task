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


        class CjrlGikRotationConstraint:public CjrlGikJointStateConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Copy
     */
    virtual CjrlGikStateConstraint* clone() const =0;

    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const=0;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual CjrlHumanoidDynamicRobot& robot() =0 ;

    /**
    \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint)=0;
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint()=0;
    /**
    \brief Set the target orientation for this constraint.
     */
    virtual void  targetOrientation(const matrix3d& inRot)=0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const matrix3d& targetOrientation()=0;


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
    virtual const vectorN& value() = 0;

    /**
    \brief Get the constraint Jacobian wrt all (internal and external) configuration variables.
    The contacts with the world are not taken into account
     */
    virtual const matrixNxP& jacobianFromRoot() = 0;

    /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
    The interaction with the environment is taken into account (for instance a foot on the ground)
     */
    virtual const matrixNxP& jacobian() = 0;

    /**
    @}
     */

    /**
    \brief Destructor
     */
    virtual ~CjrlGikRotationConstraint() =0;
};

#endif
