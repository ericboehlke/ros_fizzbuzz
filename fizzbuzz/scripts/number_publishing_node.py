#!/usr/bin/env python

import rospy

# import a number message
from std_msgs.msg import Int32

class NumberPublisherNode:
    def __init__(self):
        rospy.loginfo("Starting number_publisher")

        # create a variable to hold the current count
        self.counter = 0

        # create a publisher object to send numbers
        self.pub = rospy.Publisher("numbers", Int32, queue_size=10)

        PUBLISH_RATE = 10.0 # 10 Hz
        # create a timer that calls the timer_callback function at
        # the specified rate
        rospy.Timer(rospy.Duration(1.0/PUBLISH_RATE), self.timer_callback)

    def timer_callback(self, event):
        # this function is called by the timer

        # loginfo to print the current number to the terminal
        rospy.loginfo(self.counter)

        # publish the string message
        self.pub.publish(self.counter)

        # increment the counter
        self.counter += 1


if __name__ == "__main__":
    rospy.init_node("number_publisher")
    node = NumberPublisherNode()
    rospy.spin()
