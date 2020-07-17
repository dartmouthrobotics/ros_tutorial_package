#! /usr/bin/python

# Copyright (c) 2020 Monika Roznere, RLab @ Dartmouth College

import rospy

from ros_tutorial_package.interface import Interface

if __name__ == "__main__":
    # Initialize node.
    # Name cannot be the same name as node created in launch file
    # nor be a package name
    rospy.init_node("ros_package_example", anonymous=False)

    # Creating object that handles the ROS interface and the core of program
    interface = Interface()

    rospy.loginfo("Initialization complete.")

    # Let node spin continuously until node shutdown or user presses Ctrl + C
    interface.run()
