/*
        Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
 
        Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)
 
*/

#ifndef JRL_GIK_PLANE_CONSTRAINT_H
#define JRL_GIK_PLANE_CONSTRAINT_H

#include "jrlGikJointStateConstraint.h"


/**
\brief Description of a constraint that limits the position of a point of the robot to a given plan.
 
The constraint is defined by the following equation:
   \f{eqnarray*} \left(\vec{MT} | \vec{u}\right) = 0 \f}
   where 
   \li \f$ M\f$ is a point attached to the joint (specified in the joint's local frame).
   \li \f$ T\f$ is a point in the environment,
   \li \f$\vec{u}\f$ is a vector defining the normal to the task plane.
 
 */


class CjrlGikPlaneConstraint:public CjrlGikJointStateConstraint
{
public:
 
    /**
    \brief Set the point \f$M\f$ associated to the constraint.
     */
    virtual void  localPoint(const vector3d& inPoint) = 0;
    /**
    \brief Get the point associated to the constraint (in joint's local frame).
     */
    virtual const vector3d& localPoint() = 0;
    /**
    \brief Set a point \f$T\f$ of the target plane (in world's frame).
     */
    virtual void  worldPlanePoint(const vector3d& inPoint) = 0;
    /**
    \brief Get the point of the defined plane (in world's frame).
     */
    virtual const vector3d& worldPlanePoint() = 0;
    /**
    \brief Set the normal \f$\vec{u}\f$ of the target plane (in world's frame).
     */
    virtual void  worldPlaneNormal(const vector3d& inPoint) = 0;
    /**
    \brief Get the normal of the defined plane (in world's frame).
     */
    virtual const vector3d& worldPlaneNormal() = 0;

};

#endif
