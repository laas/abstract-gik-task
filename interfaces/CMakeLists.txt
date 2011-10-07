# Copyright 2010, 2011 Oussama Kanoun, LAAS-CNRS
#
# This file is part of abstract-gik-task.
# abstract-gik-task is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# abstract-gik-task is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# abstract-gik-task. If not, see <http://www.gnu.org/licenses/>.

INSTALL(FILES
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

	DESTINATION include/gikTask
)
