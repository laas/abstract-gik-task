dnl autoconf script for abstract interface Generalized Inverse Kinematics 
dnl Tasks.
dnl
dnl   Research carried out within the scope of the Associated International 
dnl   Laboratory: Joint Japanese-French Robotics Laboratory (JRL)
dnl
dnl   Author: Florent Lamiraux LAAS-CNRS
dnl

AC_INIT([abstractGikTask],[2.3.99],[openrobots@laas.fr])
AC_PREREQ(2.59)

AM_INIT_AUTOMAKE([foreign no-define])
AC_PROG_INSTALL

define(dyn_reqd, [abstractRobotDynamics >= 1.11])
AC_SUBST(DYN_REQD, "dyn_reqd")
PKG_CHECK_MODULES(ABS_ROBOT_DYN, dyn_reqd)
ABS_ROBOT_DYN_PREFIX=`$PKG_CONFIG abstractRobotDynamics --variable=prefix`
ABS_ROBOT_DYN_DOCDIR=`$PKG_CONFIG abstractRobotDynamics --variable=docdir`
AC_SUBST(ABS_ROBOT_DYN_PREFIX)
AC_SUBST(ABS_ROBOT_DYN_DOCDIR)

AC_OUTPUT(Makefile
	interfaces/Makefile
	documentation/Makefile
	documentation/gikTask.dox
	abstractGikTask.pc
)
