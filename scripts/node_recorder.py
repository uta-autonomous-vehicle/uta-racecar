#!/usr/bin/env python


import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import actionlib    
from utils.capture_vision_steering import Capture, AutoDriver
from drive import Drive, DriveTest

class RosNode:
    def __init__(self):
        rospy.init_node("node_recorder")
        rospy.loginfo("Starting RosNode node_recorder ")

    def initiate_recording(self):
        capture = Capture()
        capture.initiate_setup_to_record_vision()
        capture.register_callbacks_for_saving_data()


if __name__ == "__main__":
    return
    ros_node = RosNode()
    ros_node.initiate_recording()

    while not rospy.is_shutdown():
        rospy.spin()