#ifndef JRL_GIK_OBJECT_CONSTRUCTOR
#define JRL_GIK_OBJECT_CONSTRUCTOR

#include "jrlGikGazeConstraint.h"
#include "jrlGikPositionConstraint.h"
#include "jrlGikRotationConstraint.h"
#include "jrlGikTransformationConstraint.h"
#include "jrlGikParallelConstraint.h"
#include "jrlGikPlaneConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlGikWholeBodyMotionPlanner.h"


template < class CPlaneConstraint, class CParallelConstraint, class CRotationConstraint, class CPositionConstraint, class CTransformationConstraint, class CPointingConstraint, class CGazeConstraint, class CWholeBodyMotionPlanner>

class CjrlGikObjectConstructor
{
public:

    /**
        \brief Construct and return a pointer to a CjrlGikWholeBodyMotionPlanner.
        \param inRobot : associated robot.
        \param inRobot : motion sampling period.
     */
    static CjrlGikWholeBodyMotionPlanner* createWholeBodyMotionPlanner(CjrlHumanoidDynamicRobot& inRobot, double inSamplingPeriod)
    {
        return new CWholeBodyMotionPlanner(inRobot, inSamplingPeriod);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikPositionConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalPoint : point given in inJoint's frame and to be moved to inTargetWorldPoint.
        \param inTargetWorldPoint : target point in the work space.
     */
    static CjrlGikPositionConstraint* createPositionConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalPoint,const vector3d& inTargetWorldPoint)
    {
        return new CPositionConstraint(inRobot, inJoint, inLocalPoint, inTargetWorldPoint);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikPointingConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalOrigin : origin point of the pointing vector (body local frame)
        \param inLocalVector : the pointing vector (body local frame)
        \param inTargetWorldPoint : the point in world frame with which the segment defined by the origin and the pointing vector is to be aligned.
     */
    static CjrlGikPointingConstraint* createPointingConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalOrigin, const vector3d& inLocalVector, const vector3d& inTargetWorldPoint)
    {
        return new CPointingConstraint(inRobot, inJoint, inLocalOrigin,inLocalVector, inTargetWorldPoint);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikGazeConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inGazeTarget : the point in world frame where the robot is to look.
     */
    static CjrlGikGazeConstraint* createGazeConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inGazeTarget)
    {
        return new CGazeConstraint(inRobot, inJoint,inGazeTarget);
    }
    /**
        \brief Construct and return a pointer to a CjrlGikTransformationConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalPoint : point given in inJoint's frame and to be moved to inTargetWorldPoint.
        \param inTargetWorldPoint : target point in the work space.
        \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGikTransformationConstraint* createTransformationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalPoint,const vector3d& inTargetWorldPoint,const matrix3d& inTargetOrientation)
    {
        return new CTransformationConstraint(inRobot, inJoint, inLocalPoint, inTargetWorldPoint, inTargetOrientation);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikTransformationConstraint (transformation matrix)
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalPoint : point given in joint frame.
        \param inTargetWorldTransformation : target transformation.
     */
    static CjrlGikTransformationConstraint* createTransformationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalPoint,const matrix4d& inTargetWorldTransformation)
    {
        return new CTransformationConstraint( inRobot, inJoint, inLocalPoint,  inTargetWorldTransformation);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikRotationConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGikRotationConstraint* createRotationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3d& inTargetOrientation)
    {
        return new CRotationConstraint(inRobot, inJoint, inTargetOrientation);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikParallelConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
        \param inTargetWorldVector : target point in the work space.
     */
    static CjrlGikParallelConstraint* createParallelConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalVector,const vector3d& inTargetWorldVector)
    {
        return new CParallelConstraint(inRobot, inJoint,  inLocalVector, inTargetWorldVector);
    }

    /**
        \brief Construct and return a pointer to a CjrlGikPlaneConstraint
        \param inRobot : associated robot.
        \param inJoint : associated joint.
        \param inLocalPoint : point given in joint local frame to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
        \param inTargetPlanePoint : a point of the target plane in world frame
        \param inTargetPlaneNormal : a normal vector of the plane in world frame
     */
    static CjrlGikPlaneConstraint* createPlaneConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3d& inLocalPoint,const vector3d& inTargetPlanePoint, const vector3d& inTargetPlaneNormal)
    {
        return new CPlaneConstraint(inRobot, inJoint, inLocalPoint, inTargetPlanePoint, inTargetPlaneNormal);
    }
};

#endif
