# Copyright 2010, Oussama Kanoun, LAAS-CNRS
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

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Includes from jrl-cmaketools
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

# Define properties of the project
SET(PROJECT_NAME abstract-gik-task)
SET(PROJECT_DESCRIPTION "Interface to hpp-gik")
SET(PROJECT_URL "")


SET(CXX_DISABLE_WERROR True)

# Dependency to abstract-robot-dynamics
ADD_REQUIRED_DEPENDENCY(abstract-robot-dynamics >= 1.16.3)

# Initialize generic properties of the project
SETUP_PROJECT()

# Define subdirectories to explore for cmake
ADD_SUBDIRECTORY(interfaces)

# Generate dependency to abstract-gik-task in pc file
PKG_CONFIG_APPEND_LIBS("abstract-gik-task")

# Ask to generate the final steps.
SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
