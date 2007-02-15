#ifndef JRL_GIK_OBJECT_CONSTRUCTOR
#define JRL_GIK_OBJECT_CONSTRUCTOR

#include "jrlGik1DPosConstraint.h"
#include "jrlGik3DPosConstraint.h"
#include "jrlGik3DOrientConstraint.h"
#include "jrlGik6DFullConstraint.h"
#include "jrlGik2DOrientConstraint.h"
#include "jrlGikMotionConstraint.h"
#include "jrlGikWholeBodyMotionPlanner.h"

#define TPLT class Mnxp,class M4x4,class M3x3,class Vn,class V3

template < CjrlGik1DPosConstraint<TPLT>, CjrlGik2DOrientConstraint<TPLT>, CjrlGik3DOrientConstraint<TPLT>, CjrlGik3DPosConstraint<TPLT>, CjrlGik6DFullConstraint<TPLT>, CjrlGikMotionConstraint<TPLT>, CjrlGikWholeBodyMotionPlanner<TPLT> >
class CjrlGikObjectConstructor
{
public:

    /**
    \brief Construct and return a pointer to a WholeBodyMotionPlanner.
    \param inRobot : associated robot.
    \param inRobot : motion sampling period.
     */
    virtual static CjrlGikWholeBodyMotionPlanner<TPLT>* createWholeBodyMotionPlanner(CjrlHumanoidDynamicRobot<TPLT>& inRobot, double inSamplingPeriod) = 0;

    /**
    \brief Construct and return a pointer to a 3D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
     */
    virtual static CjrlGik3DPosConstraint<TPLT>* create3DPositionConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetWorldPoint) =0;
        
    /**
    \brief Construct and return a pointer to a 6D full state constraint (3D position and orientation matrix) on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldPoint : target point in the work space.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    virtual static CjrlGik6DFullConstraint<TPLT>* create6DFullConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetWorldPoint,const M3x3& inTargetOrientation) =0;
    
    /**
    \brief Construct and return a pointer to a 6D full state constraint (transformation) on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inWorldTarget.
    \param inTargetWorldTransformation : target point in the work space.
     */
    virtual static CjrlGik6DFullConstraint<TPLT>* create6DFullConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const M4x4& inTargetWorldTransformation) =0;

    /**
    \brief Construct and return a pointer to a 3D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    virtual static CjrlGik3DOrientConstraint<TPLT>* create3DOrientationConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const M3x3& inTargetOrientation) =0;
    
    /**
    \brief Construct and return a pointer to a 2D Orientation state constraint on a joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
    \param inTargetWorldVector : target point in the work space.
     */
    virtual static CjrlGik2DOrientConstraint<TPLT>* create2DOrientationConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalVector,const V3& inTargetWorldVector) =0;

    /**
    \brief Construct and return a pointer to a 1D Position state constraint on a point connected to a given joint of the robot.
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
    \param inTargetPlanePoint : a point of the target plane in world frame (see CjrlGik1DPosConstraint class definition for details).
    \param inTargetPlaneNormal : a normal vector of the plane in world frame (see CjrlGik1DPosConstraint class definition for details).
     */
    virtual static CjrlGik1DPosConstraint<TPLT>* create1DPositionConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetPlanePoint, const V3& inTargetPlaneNormal) =0;
    
    /**
    \brief Construct and return a pointer to a Motion constraint.
    \param inSamplingPeriod : the interval of time between two successsive state constraints.
     */
    virtual static CjrlGikMotionConstraint<TPLT>* createMotionConstraint(double inSamplingPeriod) =0;
};

#endif
