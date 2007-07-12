dnl autoconf script for abstract interface Generalized Inverse Kinematics 
dnl Tasks.
dnl
dnl   Research carried out within the scope of the Associated International 
dnl   Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([abstractGikTask],[1.0],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE
AC_PROG_INSTALL

PKG_CHECK_MODULES(ABS_ROBOT_DYN, [abstractRobotDynamics])
ABS_ROBOT_DYN_PREFIX=`$PKG_CONFIG abstractRobotDynamics --variable=prefix`
AC_SUBST(ABS_ROBOT_DYN_PREFIX)
AC_SUBST(ABS_ROBOT_DYN_CFLAGS)
AC_SUBST(ABS_ROBOT_DYN_LIBS)

AC_OUTPUT(Makefile
	interfaces/Makefile
	documentation/Makefile
	documentation/gikTask.dox
	abstractGikTask.pc
)
