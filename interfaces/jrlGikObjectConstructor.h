#ifndef JRL_GIK_OBJECT_CONSTRUCTOR
#define JRL_GIK_OBJECT_CONSTRUCTOR

#include "jrlGik/jrlGik1DPosConstraint.h"
#include "jrlGik/jrlGik3DPosConstraint.h"
#include "jrlGik/jrlGik3DOrientConstraint.h"
#include "jrlGik/jrlGik2DOrientConstraint.h"
#include "jrlGik/jrlGikMotionConstraint.h"
#include "jrlGik/jrlGikWholeBodyMotionPlanner.h"

template <CjrlGik1DPosConstraint, CjrlGik2DOrientConstraint, CjrlGik3DOrientConstraint, CjrlGik3DPosConstraint, CjrlGikMotionConstraint, CjrlGikWholeBodyMotionPlanner>
class CjrlGikObjectConstructor
{
public:

    /**
    \brief Construct and return a pointer to a WholeBodyMotionPlanner.
    \param inRobot : associated robot.
     */
    static CjrlGikWholeBodyMotionPlanner* createWholeBodyMotionPlanner(const CjrlHumanoidDynamicRobot& inRobot)
    {
        return new CjrlGikWholeBodyMotionPlanner(inRobot);
    };

    /**
    \brief Construct and return a pointer to a 3D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
     */
    static CjrlGik3DPosConstraint* create3DPositionConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetWorldPoint)
    {
        return new CjrlGik3DPosConstraint(inRobot, inJoint, inLocalPoint, inTargetWorldPoint);
    };

    /**
    \brief Construct and return a pointer to a 3D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGik3DOrientConstraint* create3DOrientationConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint, const matrix3& inTargetOrientation)
    {
        return new CjrlGik3DOrientConstraint(inRobot, inJoint, inTargetOrientation);
    };

    /**
    \brief Construct and return a pointer to a 2D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
    \param inTargetWorldVector : target point in the work space.
     */
    static CjrlGik2DOrientConstraint* create2DOrientationConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint,, const vector3& inLocalVector,const vector3& inTargetWorldVector)
    {
        return new CjrlGik2DOrientConstraint(inRobot, inJoint,  inLocalVector, inTargetWorldVector);
    };

    /**
    \brief Construct and return a pointer to a 1D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
    \param inTargetPlanePoint : a point of the target plane in world frame (see CjrlGik1DPosConstraint class definition for details).
    \param inTargetPlaneNormal : a normal vector of the plane in world frame (see CjrlGik1DPosConstraint class definition for details).
     */
    static CjrlGik1DPosConstraint* create1DPositionConstraint(const CjrlHumanoidDynamicRobot& inRobot, const CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetPlanePoint, const vector3& inTargetPlaneNormal)
    {
        return new CjrlGik1DPosConstraint(inRobot, inJoint, inLocalPoint, inTargetPlanePoint, inTargetPlaneNormal);
    };
    
    /**
    \brief Construct and return a pointer to a Motion constraint.
    \param inSamplingPeriod : the interval of time between two successsive state constraints.
     */
    static CjrlGikMotionConstraint* createMotionConstraint(double inSamplingPeriod)
    {
        return new CjrlGikMotionConstraint(inSamplingPeriod);
    };
};

#endif
