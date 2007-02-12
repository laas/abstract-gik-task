#ifndef JRL_GIK_OBJECT_CONSTRUCTOR
#define JRL_GIK_OBJECT_CONSTRUCTOR

#include "jrlGik1DPosConstraint.h"
#include "jrlGik3DPosConstraint.h"
#include "jrlGik3DOrientConstraint.h"
#include "jrlGik2DOrientConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlGikWholeBodyMotionPlanner.h"

#include "hppGik1DPosConstraint.h"
#include "hppGik3DPosConstraint.h"
#include "hppGik3DOrientConstraint.h"
#include "hppGik2DOrientConstraint.h"
#include "hppGikMotionConstraint.h"
#include "hppGikWhithinOneStepMotionPlanner.h"

template <ChppGik1DPosConstraint, ChppGik2DOrientConstraint, ChppGik3DOrientConstraint, ChppGik3DPosConstraint, ChppGikMotionConstraint, ChppGikWholeBodyMotionPlanner>
class CjrlGikObjectConstructor
{
public:

    /**
    \brief Construct and return a pointer to a WholeBodyMotionPlanner.
    \param inRobot : associated robot.
    \param inRobot : motion sampling period.
     */
    static CjrlGikWholeBodyMotionPlanner* createWholeBodyMotionPlanner(CjrlHumanoidDynamicRobot& inRobot, double inSamplingPeriod)
    {
        return new ChppGikWhithinOneStepMotionPlanner(inRobot, inSamplingPeriod);
    };

    /**
    \brief Construct and return a pointer to a 3D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
     */
    static CjrlGik3DPosConstraint* create3DPositionConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetWorldPoint)
    {
        return new ChppGik3DPosConstraint(inRobot, inJoint, inLocalPoint, inTargetWorldPoint);
    };

    /**
    \brief Construct and return a pointer to a 3D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGik3DOrientConstraint* create3DOrientationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3& inTargetOrientation)
    {
        return new ChppGik3DOrientConstraint(inRobot, inJoint, inTargetOrientation);
    };

    /**
    \brief Construct and return a pointer to a 2D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
    \param inTargetWorldVector : target point in the work space.
     */
    static CjrlGik2DOrientConstraint* create2DOrientationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint,, const vector3& inLocalVector,const vector3& inTargetWorldVector)
    {
        return new ChppGik2DOrientConstraint(inRobot, inJoint,  inLocalVector, inTargetWorldVector);
    };

    /**
    \brief Construct and return a pointer to a 1D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
    \param inTargetPlanePoint : a point of the target plane in world frame (see CjrlGik1DPosConstraint class definition for details).
    \param inTargetPlaneNormal : a normal vector of the plane in world frame (see CjrlGik1DPosConstraint class definition for details).
     */
    static CjrlGik1DPosConstraint* create1DPositionConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetPlanePoint, const vector3& inTargetPlaneNormal)
    {
        return new ChppGik1DPosConstraint(inRobot, inJoint, inLocalPoint, inTargetPlanePoint, inTargetPlaneNormal);
    };
    
    /**
    \brief Construct and return a pointer to a Motion constraint.
    \param inSamplingPeriod : the interval of time between two successsive state constraints.
     */
    static CjrlGikMotionConstraint* createMotionConstraint(double inSamplingPeriod)
    {
        return new ChppGikMotionConstraint(inSamplingPeriod);
    };
};

#endif
