#!/usr/bin/python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from utils import Capture, AutoDriver, BaseImageManager
from drive import Drive, DriveTest
import actionlib    

if __name__ == "__main__":
    rospy.init_node("uta_racecar")

    c = Capture()
    c.register_callbacks_for_saving_data()

    while not rospy.is_shutdown():
        rospy.spin()