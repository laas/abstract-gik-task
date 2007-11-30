/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_PARALLEL_CONSTRAINT_H
#define JRL_GIK_PARALLEL_CONSTRAINT_H

#include "jrlGikJointStateConstraint.h"

/**
\brief Constraint on a vector in a joint frame to be parallel to a vector given in world frame.
        
The constraint is defined by the following equation:
   \f{eqnarray*}   \vec{R}\times \vec{R}_T = 0   \f}
   where 
   \li \f$\vec{R}\f$ is a vector in the joint frame (specified in the joint's local frame).
   \li \f$\vec{R}_T\f$ is a vector in the global frame.
 */


class CjrlGikParallelConstraint:virtual public CjrlGikJointStateConstraint
{
public:
    /**
    \brief Set the vector \f$\vec{R}\f$ associated to the constraint  (in joint's local frame).
     */
    virtual void  localVector(const vector3d& inVector)=0;
    /**
    \brief Get the vector associated to the constraint (in joint's local frame).
     */
    virtual const vector3d& localVector()=0;
    /**
    \brief Set the target vector \f$\vec{R}_T\f$ associated to the constraint.
     */
    virtual void  targetVector(const vector3d& inVector)=0;
    /**
    \brief Get the target vector associated to the constraint.
     */
    virtual const vector3d& targetVector()=0;

};

#endif
