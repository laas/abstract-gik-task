/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_6DFULL_CONSTRAINT_H
#define JRL_GIK_6DFULL_CONSTRAINT_H

#include "jrlGikStateConstraint.h"

/**
\brief Specify a position and orientation constraint on a body of the robot.
 */
template <class Mnxp,class M4x4,class M3x3,class Vn,class V3>
class CjrlGik6DFullConstraint:public CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>
{
public:
    /**
    \name Definition of the constraint
    @{
     */

    /**
    \brief Copy
     */
    virtual CjrlGikStateConstraint<Mnxp,M4x4,M3x3,Vn,V3>* clone() const =0;

    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const = 0;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual CjrlHumanoidDynamicRobot<Mnxp,M4x4,M3x3,Vn,V3>& robot() =0 ;

    /**
    \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* inJoint) = 0;
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint<Mnxp,M4x4,M3x3,Vn,V3>* joint() = 0;
    /**
    \brief Set the point (in joint's local frame) associated to the constraint.
     */
    virtual void  localPoint(const V3& inPoint) = 0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const V3& localPoint() = 0;
    /**
    \brief Set the target point associated to the constraint (in world's frame).
     */
    virtual void  worldTarget(const V3& inPoint) = 0;
    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const V3& worldTarget() = 0;
    /**
    \brief Set the target orientation for this constraint.
     */
    virtual void  targetOrientation(const M3x3& inRot)=0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const M3x3& targetOrientation()=0;

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
    /**
    \brief Destructor
     */
    virtual ~CjrlGik6DFullConstraint() = 0;

};

#endif
