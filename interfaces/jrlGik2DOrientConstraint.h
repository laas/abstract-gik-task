/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_2DORIENT_CONSTRAINT_H
#define JRL_GIK_2DORIENT_CONSTRAINT_H

#include "jrlGik/jrlGikStateConstraint.h"
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/matrix.hpp"


namespace ublas = boost::numeric::ublas;

//temporary typedefs. They'll be deleted when having the proper linkage with the walkGen's algebra classes
typedef std::vector<double> vector3;
//temporary

/**
\brief Specify a 2D orientation constraint on a body of the robot.
        
The constraint is defined by the following equation:
   \f{eqnarray*}   \vec{R}\times \vec{R}_T = 0   \f}
   where 
   \li \f$\vec{R}\f$ is a vector in the joint frame (specified in the joint's local frame).
   \li \f$\vec{R}_T\f$ is a vector in the global frame.
 */

class CjrlGik2DOrientConstraint:public CjrlGikStateConstraint
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
    \brief Set the vector \f$\vec{R}\f$ associated to the constraint  (in joint's local frame).
     */
    virtual void  localVector(const vector3& inVector)=0;
    /**
    \brief Get the vector associated to the constraint (in joint's local frame).
     */
    virtual const vector3& localVector()=0;
    /**
    \brief Set the target vector \f$\vec{R}_T\f$ associated to the constraint.
     */
    virtual void  targetVector(const vector3& inVector)=0;
    /**
    \brief Get the target vector associated to the constraint.
     */
    virtual const vector3& targetVector()=0;


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
    virtual const ublas::vector<double>& value() = 0;

    /**
    \brief Get the constraint Jacobian wrt all (internal and external) configuration variables.
    The contacts with the world are not taken into account
     */
    virtual const ublas::matrix<double>& jacobianFromRoot() = 0;

    /**
    \brief Get the constraint Jacobian wrt internal configuration variables.
    The interaction with the environment is taken into account (for instance a foot on the ground)
     */
    virtual const ublas::matrix<double>& jacobian() = 0;

    /**
    @}
     */

    /**
    \brief Destructor
     */
    virtual ~CjrlGik2DOrientConstraint() =0;

};

#endif
