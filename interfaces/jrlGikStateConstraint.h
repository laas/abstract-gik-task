/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
  Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_STATE_CONSTRAINT_H
#define JRL_STATE_CONSTRAINT_H

#include "abstract-robot-dynamics/humanoid-dynamic-robot.hh"
#include "jrlLinearConstraint.h"

/**
\brief Specify a linear constraint over the state of a humanoid robot.
*/
class CjrlGikStateConstraint: virtual public CjrlLinearConstraint
{
public:

    /**
    \brief Copy constructor.
     */
    virtual CjrlGikStateConstraint* clone() const =0;

    /**
    \name Definition of the constraint.
    @{
    */

    /**
    \brief Get associated robot.
     */
    virtual CjrlDynamicRobot& robot() = 0;

    /**
    \brief Select the joint in the robot that serves as root for computation of jacobians.
     */
    virtual void jacobianRoot(CjrlJoint& inJoint) =0 ;

    /**
    \brief Get the influencing dofs vector which is a binary vector whose size matches the robot cnfiguration's, where an element with value 1 indicates that the corresponding degree of freedom can modify the value of this constraint and an element with value 0 cannot.
     */
    virtual vectorN& influencingDofs() = 0;

    /**
    @}
    */

    /**
    \name Computations
    @{
     */

    /**
    \brief Compute the influencing dofs vector.
     */
    virtual void computeInfluencingDofs() = 0;

    /**
    \brief Compute the value of the constraint.
     */
    virtual void computeValue() = 0;

    /**
    \brief Compute the Jacobian of the constraint with respect to the internal degrees of freedom and to a selected root joint (see method CjrlGikStateConstraint::jacobianRoot()).
     */
    virtual void computeJacobian() = 0;

    /**
    @}
     */
};



#endif
