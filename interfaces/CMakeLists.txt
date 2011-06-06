SET(${PROJECT_NAME}_HEADERS
	jrlGikGazeConstraint.h
	jrlGikComConstraint.h
	jrlGikJointStateConstraint.h
	jrlGikMotionConstraint.h
	jrlGikObjectConstructor.h
	jrlGikParallelConstraint.h
	jrlGikPlaneConstraint.h
	jrlGikPointingConstraint.h
	jrlGikPositionConstraint.h
	jrlGikRotationConstraint.h
	jrlGikStateConstraint.h
	jrlLinearConstraint.h
	jrlGikTransformationConstraint.h
	jrlRobotMotion.h
)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION include/gikTask
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
