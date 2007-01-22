#ifndef JRL_GIK_STATE_SUBTASK_H
#define JRL_GIK_STATE_SUBTASK_H

#include <stdlib.h>
#include "jrlGik/jrlGikStateConstraint.h"
#include "jrlRobot/jrlJoint.h"
#include "jrlRobot/jrlHumanoidDynamicRobot.h"

#include <boost/numeric/ublas/matrix.hpp>

/**
   \brief A subtask is a set of constraints on the robot that share the same priority. The priority is a positive integer that defines the rank of resolution of the subtask within the generalized kinematics framework. The higher the priority the smaller the integer should be.
 */
class CjrlGikWholeBodyStateSubtask:CjrlGikStateConstraint
{

public:

    /**
        \brief Get the priority.
     */
    virtual unsigned int priority() =0;

    /**
        \brief Set the priority.
     */
    virtual void priority(unsigned int inPriority) =0;

    /**
        \brief Select the joints that will be used to achieve this subtask.
     */
    virtual void workingJoints(std::vector<CjrlJoint*>& inJoints)=0;
    /**
    \brief Return the mask on the configuration that defines the joints working for this subtask.
     */
    virtual const std::vector<CjrlJoint*>& workingJoints()=0;
    /**
        \brief Add a constraint.
     */
    virtual void addConstraint(CjrlGikStateConstraint* inJrlStateConstraint)=0;
    /**
        \brief Remove a constraint
     */
    virtual bool removeConstraint(CjrlGikStateConstraint* inJrlStateConstraint)=0;
    /**
        \brief Get a pointer to the constraint at the given rank
     */
    virtual CjrlGikStateConstraint* constraintAtRank(unsigned int inRank)=0;
    /**
        \brief Get the number of constraints in this subtask
     */
    virtual unsigned int numberConstraints()=0;

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
    virtual ~CjrlGikWholeBodyStateSubtask()= 0;

};

#endif
