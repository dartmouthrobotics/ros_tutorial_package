cmake_minimum_required(VERSION 2.8.3)

# Project name must be the same name as the package.
project(ros_tutorial_package)

find_package(catkin REQUIRED
  rospy     # Required if using ROS
  std_msgs
  roslint
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# roslint is useful to help clean formatting errors (check readme for explanation)
roslint_python(
  scripts/start_example.py
  src/${PROJECT_NAME}/interface.py
  src/${PROJECT_NAME}/math_lib.py
)

catkin_install_python(PROGRAMS
   scripts/start_example.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
