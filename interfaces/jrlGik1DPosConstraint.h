/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_1DPOS_CONSTRAINT_H
#define JRL_GIK_1DPOS_CONSTRAINT_H

#include "jrlGik/jrlGikStateConstraint.h"
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/matrix.hpp"


namespace ublas = boost::numeric::ublas;

//temporary typedefs. They'll be deleted when having the proper linkage with the walkGen's algebra classes
typedef std::vector<double> vector3;
//temporary

/**
\brief Description of a constraint that limits the position of a point of the robot to a given plan.
 
The constraint is defined by the following equation:
   \f{eqnarray*} \left(\vec{MT} | \vec{u}\right) = 0 \f}
   where 
   \li \f$ M\f$ is a point attached to the joint (specified in the joint's local frame).
   \li \f$ T\f$ is a point in the environment,
   \li \f$\vec{u}\f$ is a vector defining the normal to the task plane.
 
 */


class CjrlGik1DPosConstraint:CjrlGikStateConstraint
{
public:
    /**
    \name Definition of the constraint
    @{
     */
    
    /**
    \brief Constructor
     */
    virtual CjrlGik1DPosConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetPlanePoint, const vector3& inTargetPlaneNormal);
    /**
    \brief Get the dimension of the constraint.
     */
    virtual unsigned int dimension() const = 0;

    /**
    \brief Get robot associated to the constraint.
     */
    virtual const CjrlHumanoidDynamicRobot& robot() = 0;

    /**
    \brief Set the joint associated to the constraint.
     */
    virtual void  joint(CjrlJoint* inJoint) = 0;
    /**
    \brief Get the joint associated to the constraint.
     */
    virtual  CjrlJoint* joint() = 0;
    /**
    \brief Set the point \f$M\f$ associated to the constraint.
     */
    virtual void  localPoint(vector3 inPoint) = 0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const vector3& localPoint() = 0;
    /**
    \brief Set a point \f$T\f$ of the target plane (in world's frame).
     */
    virtual void  worldPlanePoint(const vector3& inPoint) = 0;
    /**
    \brief Get the point of the defined plane (in world's frame).
     */
    virtual const vector3& worldPlanePoint() = 0;
    /**
    \brief Set the normal \f$\vec{u}\f$ of the target plane (in world's frame).
     */
    virtual void  worldPlaneNormal(const vector3& inPoint) = 0;
    /**
    \brief Get the normal of the defined plane (in world's frame).
     */
    virtual const vector3& worldPlaneNormal() = 0;

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
    virtual ~CjrlGik1DPosConstraint() = 0;

};

#endif
