#ifndef JRL_GIK_GAZE_CONSTRAINT_H
#define JRL_GIK_GAZE_CONSTRAINT_H

#include "jrlGikPointingConstraint.h"

/**
\brief This is a PointingConstraint whose joint is the humanoid robot's gaze joint. Only the target point in the world is provided.
 */
class CjrlGikGazeConstraint:public CjrlGikPointingConstraint
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
        virtual unsigned int dimension() const = 0;

    /**
        \brief Get robot associated to the constraint.
     */
        virtual CjrlHumanoidDynamicRobot& robot() =0 ;

    /**
        \brief Set the joint associated to the constraint (illelgal operation for gaze constraint).
     */
        void  joint(CjrlJoint* inJoint) {};
    /**
        \brief Get the joint associated to the constraint (should return the robot gaze joint).
     */
        virtual  CjrlJoint* joint() = 0;
    /**
        \brief Set the origin of the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
        virtual void  localOrigin(const vector3d& inPoint) = 0;
    /**
        \brief Get the origin of the pointing vector.
     */
        virtual const vector3d& localOrigin() = 0;
    /**
        \brief Set the pointing vector in joint's local frame (illelgal operation for gaze constraint).
     */
        void  localVector(const vector3d& inPoint) {};
    /**
        \brief Set the pointing vector in joint's local frame
     */
        virtual const vector3d& localVector() = 0;    
    /**
        \brief Set the target point associated to the constraint (in world's frame).
     */
        virtual void  worldTarget(const vector3d& inPoint) = 0;
    /**
        \brief Get the target point associated to the constraint (in world's frame).
     */
        virtual const vector3d& worldTarget() = 0;

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
        virtual ~CjrlGikGazeConstraint() = 0;

};

#endif
