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

#define TPLT class Mnxp,class M4x4,class M3x3,class Vn,class V3

template < CjrlGikPlaneConstraint<TPLT>, CjrlGikParallelConstraint<TPLT>, CjrlGikRotationConstraint<TPLT>, CjrlGikPositionConstraint<TPLT>, CjrlGikTransformationConstraint<TPLT>, CjrlGikWholeBodyMotionPlanner<TPLT>, CjrlGikPointingConstraint<TPLT>, CjrlGikGazeConstraint<TPLT> >
class CjrlGikObjectConstructor
{
public:

    /**
    \brief Construct and return a pointer to a CjrlGikWholeBodyMotionPlanner.
    \param inRobot : associated robot.
    \param inRobot : motion sampling period.
     */
    virtual static CjrlGikWholeBodyMotionPlanner<TPLT>* createWholeBodyMotionPlanner(CjrlHumanoidDynamicRobot<TPLT>& inRobot, double inSamplingPeriod) = 0;

    /**
    \brief Construct and return a pointer to a CjrlGikPositionConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inTargetWorldPoint.
    \param inTargetWorldPoint : target point in the work space.
     */
    virtual static CjrlGikPositionConstraint<TPLT>* createPositionConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetWorldPoint) =0;
    
    /**
    \brief Construct and return a pointer to a CjrlGikPointingConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalOrigin : origin point of the pointing vector (body local frame)
    \param inLocalVector : the pointing vector (body local frame)
    \param inTargetWorldPoint : the point in world frame with which the segment defined by the origin and the pointing vector is to be aligned.
     */
    virtual static CjrlGikPointingConstraint<TPLT>* createPointingConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalOrigin, const V3& inLocalVector, const V3& inTargetWorldPoint) =0;
    
    /**
    \brief Construct and return a pointer to a CjrlGikGazeConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inGazeTarget : the point in world frame where the robot is to look.
     */
    virtual static CjrlGikGazeConstraint<TPLT>* createGazeConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inGazeTarget) =0;
        
    /**
    \brief Construct and return a pointer to a CjrlGikTransformationConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in inJoint's frame and to be moved to inTargetWorldPoint.
    \param inTargetWorldPoint : target point in the work space.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    virtual static CjrlGikTransformationConstraint<TPLT>* createTransformationConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetWorldPoint,const M3x3& inTargetOrientation) =0;
    
    /**
    \brief Construct and return a pointer to a CjrlGikTransformationConstraint (transformation matrix)
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in joint frame.
    \param inTargetWorldTransformation : target transformation.
     */
    virtual static CjrlGikTransformationConstraint<TPLT>* createTransformationConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const M4x4& inTargetWorldTransformation) =0;

    /**
    \brief Construct and return a pointer to a CjrlGikRotationConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inTargetOrientation : target orientation matrix in world frame
     */
    virtual static CjrlGikRotationConstraint<TPLT>* createRotationConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const M3x3& inTargetOrientation) =0;
    
    /**
    \brief Construct and return a pointer to a CjrlGikParallelConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalVector : vector given in inJoint's frame and to be aligned with inTargetWorldVector.
    \param inTargetWorldVector : target point in the work space.
     */
    virtual static CjrlGikParallelConstraint<TPLT>* createParallelConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalVector,const V3& inTargetWorldVector) =0;

    /**
    \brief Construct and return a pointer to a CjrlGikPlaneConstraint
    \param inRobot : associated robot.
    \param inJoint : associated joint.
    \param inLocalPoint : point given in joint local frame to be moved to the plane specified by inTargetPlanePoint and inTargetPlaneNormal.
    \param inTargetPlanePoint : a point of the target plane in world frame
    \param inTargetPlaneNormal : a normal vector of the plane in world frame
     */
    virtual static CjrlGikPlaneConstraint<TPLT>* createPlaneConstraint(CjrlHumanoidDynamicRobot<TPLT>& inRobot, CjrlJoint<TPLT>& inJoint, const V3& inLocalPoint,const V3& inTargetPlanePoint, const V3& inTargetPlaneNormal) =0;
};

#endif
