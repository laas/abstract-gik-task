/*
  Research carried out within the scope of the Associated International Laboratory: Joint Japanese-French Robotics Laboratory (JRL)

  Authors: Florent Lamiraux and Oussama Kanoun (LAAS-CNRS)

*/

#ifndef JRL_LINEAR_CONSTRAINT_H
#define JRL_LINEAR_CONSTRAINT_H

/**
\brief A linear constraint is an object that returns a m-by-n matrix and an m-vector defining a linear system of equality of inequality constraints (determined by context).
 */
class CjrlLinearConstraint
{
public:
    /**
    \brief Get the dimension of the constraint.
      */
    virtual unsigned int dimension() const = 0;

    /**
    \brief Get the constraint value.
     */
    virtual const vectorN& value() = 0;

    /**
    \brief Get the constraint Jacobian
     */
    virtual const matrixNxP& jacobian() = 0;
    /**
    \brief Destructor
    */
    virtual ~CjrlLinearConstraint()
    {}
    ;
};

#end