#ifndef JRL_GIK_OBJECT_CONSTRUCTOR
#define JRL_GIK_OBJECT_CONSTRUCTOR

#include "jrlGik1DPosConstraint.h"
#include "jrlGik3DPosConstraint.h"
#include "jrlGik3DOrientConstraint.h"
#include "jrlGik6DFullConstraint.h"
#include "jrlGik2DOrientConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlGikWholeBodyMotionPlanner.h"

template <CjrlGik1DPosConstraint, CjrlGik2DOrientConstraint, CjrlGik3DOrientConstraint, CjrlGik3DPosConstraint, CjrlGik6DFullConstraint, CjrlGikMotionConstraint, CjrlGikWholeBodyMotionPlanner>
class CjrlGikObjectConstructor
{
public:

    /**
    \brief Construct and return a pointer to a WholeBodyMotionPlanner.
    \param inRobot : associated robot.
    \param inRobot : motion sampling period.
     */
    static CjrlGikWholeBodyMotionPlanner* createWholeBodyMotionPlanner(CjrlHumanoidDynamicRobot& inRobot, double inSamplingPeriod);

    /**
    \brief Construct and return a pointer to a 3D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
     */
    static CjrlGik3DPosConstraint* create3DPositionConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetWorldPoint);
        
    /**
    \brief Construct and return a pointer to a 6D full state constraint (3D position and orientation matrix) on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGik6DFullConstraint* create6DFullConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetWorldPoint,const matrix3& inTargetOrientation);
    
    /**
    \brief Construct and return a pointer to a 6D full state constraint (transformation) on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldTransformation : target point in the work space.
     */
    static CjrlGik6DFullConstraint* create6DFullConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const matrix4& inTargetWorldTransformation);

    /**
    \brief Construct and return a pointer to a 3D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    static CjrlGik3DOrientConstraint* create3DOrientationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const matrix3& inTargetOrientation);
    
    /**
    \brief Construct and return a pointer to a 2D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
    \param inTargetWorldVector : target point in the work space.
     */
    static CjrlGik2DOrientConstraint* create2DOrientationConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint,, const vector3& inLocalVector,const vector3& inTargetWorldVector);

    /**
    \brief Construct and return a pointer to a 1D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
    \param inTargetPlanePoint : a point of the target plane in world frame (see CjrlGik1DPosConstraint class definition for details).
    \param inTargetPlaneNormal : a normal vector of the plane in world frame (see CjrlGik1DPosConstraint class definition for details).
     */
    static CjrlGik1DPosConstraint* create1DPositionConstraint(CjrlHumanoidDynamicRobot& inRobot, CjrlJoint& inJoint, const vector3& inLocalPoint,const vector3& inTargetPlanePoint, const vector3& inTargetPlaneNormal);
    
    /**
    \brief Construct and return a pointer to a Motion constraint.
    \param inSamplingPeriod : the interval of time between two successsive state constraints.
     */
    static CjrlGikMotionConstraint* createMotionConstraint(double inSamplingPeriod);
};

#endif
