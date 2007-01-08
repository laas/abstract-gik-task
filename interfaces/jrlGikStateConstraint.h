/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)

*/

#ifndef JRL_STATE_CONSTRAINT_H
#define JRL_STATE_CONSTRAINT_H

#include "jrlHumanoidDynamicRobot.h"

/**
   \brief Specify a Constraint over the state of a humanoid robot.
*/

class CjrlGikStateConstraint {
public:
  /**
     \name Definition of the constraint
     @{
  */
  /**
     \brief Constructor
  */
  CjrlGikStateConstraint(const CjrlHumanoidDynamicRobot& inRobot);

  /**
     \brief Copy constructor
  */
  CjrlGikStateConstraint(const CjrlGikStateConstraint& inStateConstraint);

  /**
     \brief Get the dimension of the constraint.
  */
  virtual unsigned int dimension() const = 0;
  
  /**
     \brief Get robot associated to the constraint.
  */
  virtual const CjrlHumanoidDynamicRobot& robot() = 0;

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
  virtual void computeValue()=0;
  
  /**
     \brief Compute the Jacobian matrix of the constraint value wrt internal configuration variables.

     \note internal configuration variables exclude position of root joint in space.
  */
  virtual void computeJacobianFromRoot(ublas::matrix<double>& outJacobian)=0;

  /**
     \brief Compute the Jacobian matrix of the constraint value wrt all the configuration variables.

  */
  virtual void computeJacobian(ublas::matrix<double>& outJacobian)=0;

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
     \brief Get the constraint Jacobian wrt internal configuration variables.
  */
  virtual ublas::matrix<double> jacobianFromRoot() = 0;

  /**
     \brief Get the constraint Jacobian wrt all the configuration variables.
  */
  virtual ublas::matrix<double> jacobian() = 0;

  /**
     @}
  */
  /**
     \name Comparison
     @{
  */

  /** 
      \brief Joint comparison
      \param inConstraint
      \return true if both constraints are associated to the same joint. Note that some constraints have no associated joint.
  */
  virtual bool sameJoint(const CjrlGikStateConstraint& inConstraint) const = 0;
  /**
     @}
  */

};



#endif
