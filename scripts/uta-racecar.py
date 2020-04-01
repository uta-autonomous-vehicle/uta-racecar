#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import actionlib    
from utils.capture_vision_steering import Capture
from drive import Drive

# print "............"
def joy_callback(msg):
    # TODO: subscribed to /joy
    pass

# change_time = rospy.Time.now()

def get_time_passed(started = None):
    if not started:
        return 0
    
    return time.time() - started

def package_init(message = ""):
    print message


if __name__ == "__main__":
    package_init("uta-racecar started")
    rospy.init_node("uta_racecar")

    capture = Capture()
    # capture.initiate_setup()
    # capture.activate_listener_for_saving_data()
    capture.activate_listener_for_autonomy()

    # drive = Drive()
    # drive.test_steering()

    # drive = Drive() 
    # drive.initiate_threads()
    # drive.go_left()
    # drive.go_right()
    
    while not rospy.is_shutdown():
        rospy.spin()
