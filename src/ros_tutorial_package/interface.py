# Copyright (c) 2020 Monika Roznere, RLab @ Dartmouth College

import rospy        # used for all ROS
import rospkg       # used to retrieve path names for files in ROS packages

# message types
from std_msgs.msg import Int64

import math_lib     # custom python library file in tutorial_package


class Interface:

    def __init__(self):
        """
        Initializing variables.
        """
        # Not used, but this is the format for retrieving file paths via ROS
        rospack = rospkg.RosPack()
        if rospy.has_param('out_file'):
            self.OUT_FILE = rospack.get_path('ros_tutorial_package') + "/data/" + rospy.get_param('out_file')
        else:
            self.OUT_FILE = rospack.get_path('ros_tutorial_package') + "/data/test.txt"
        rospy.loginfo("ROS param 'out_file' set to: " + str(self.OUT_FILE))

        # Clear contents of out file.
        open(self.OUT_FILE, 'w').close()

        # Set this example tutorial node's rate
        if rospy.has_param('rate'):
            self.RATE = rospy.Rate(rospy.get_param('rate'))
            rospy.loginfo("ROS param 'rate' set to: " + str(rospy.get_param('rate')))
        else:
            self.RATE = rospy.Rate(10)
            rospy.loginfo("ROS param 'rate' set to: 10")

        """
        Setting up subscribers and publishers.
        """
        self.random_num_pub = rospy.Publisher("random_number", Int64, queue_size=1)

        rospy.Subscriber('random_number', Int64, self.random_num_callback)

        """
        You may create your own shutdown handler. This is important if a robot
        is moving, and you want to make sure it stops before ending the program.
        """
        rospy.on_shutdown(self.shutdown)

    def random_num_callback(self, msg):
        """
        Callback for retrieving the random number message.

        Args:
            msg: Int64 random number
        """

        rospy.loginfo("'random number' : " + str(msg.data))

    def run(self):
        """
        Core of program here. Will continuously run until node is killed.
        """
        while not rospy.is_shutdown():
            msg = Int64()
            """
            Make it a habit to add header information to your messages. However,
            this message type does not have a header parameter.
            """
            # msg.header.frame_id = "none"
            # msg.header.stamp = rospy.Time()
            msg.data = math_lib.random_number()

            self.random_num_pub.publish(msg)

            """
            Sleep until end of cycle. Cycle time is based on Rate. This also
            gives time for the message to publish sucessfully.
            """
            self.RATE.sleep()

    def shutdown(self):
        """
        Shutdown function that handles everything right before ending the program.
        It is called when the user presses Ctrl-C.
        """

        rospy.loginfo("Example shutting down.")
