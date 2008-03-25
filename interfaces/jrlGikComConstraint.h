#ifndef JRL_GIK_COM_CONSTRAINT_H
#define JRL_GIK_COM_CONSTRAINT_H

#include "jrlGikStateConstraint.h"



/**
\brief Specify a full or partial position constraint on the center of mass. Default target position is two-dimensional and is defined by X and Y in world frame.
 */
class CjrlGikComConstraint:public CjrlGikStateConstraint
{
public:

    /**
        \brief Set a constraint on Xcom and Ycom (in world's frame).
     */
        virtual void  targetXY(double inX, double inY) = 0;

    /**
        \brief Set constraint on x,y,z of com (in world's frame).
     */
        virtual void  targetXYZ(const vectorN& inTarget) = 0;

    /**
        \brief Get the target point associated to the constraint (in world's frame).
        Returned vector can be either 2D or 3D depending on assigned target
     */
        virtual const vectorN& worldTarget() = 0;

};

#endif
