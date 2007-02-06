/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_3DPOS_CONSTRAINT_H
#define JRL_GIK_3DPOS_CONSTRAINT_H

#include "jrlGik/jrlGikStateConstraint.h"
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/matrix.hpp"


namespace ublas = boost::numeric::ublas;

//temporary typedefs. They'll be deleted when having the proper linkage with the walkGen's algebra classes
typedef std::vector<double> vector3;
//temporary

/**
\brief Specify a 3D position constraint on a point of the robot.
 */

class CjrlGik3DPosConstraint:CjrlGikStateConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */
    
    /**
    \brief Constructor
     */
    virtual CjrlGik3DPosConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint, const vector3& inLocalPoint, const vector3& inTargetWorldPoint);
            
    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const = 0;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual const CjrlHumanoidDynamicRobot& robot()const =0 ;

    /**
    \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint) = 0;
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint() = 0;
    /**
    \brief Set the point (in joint's local frame) associated to the constraint.
     */
    virtual void  localPoint(const vector3& inPoint) = 0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const vector3& localPoint() = 0;
    /**
    \brief Set the target point associated to the constraint (in world's frame).
     */
    virtual void  worldTarget(const vector3& inPoint) = 0;
    /**
    \brief Get the target point associated to the constraint (in world's frame).
     */
    virtual const vector3& worldTarget() = 0;

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
    virtual ublas::vector<double> value() = 0;

    /**
    \brief Get the constraint Jacobian wrt all (internal and external) configuration variables.
    The contacts with the world are not taken into account
     */
    virtual ublas::matrix<double> jacobianFromRoot() = 0;

    /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
    The interaction with the environment is taken into account (for instance a foot on the ground)
     */
    virtual ublas::matrix<double> jacobian() = 0;

    /**
    @}
     */
    /**
    \brief Destructor
     */
    virtual ~CjrlGik3DPosConstraint() = 0;

};

#endif
