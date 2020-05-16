#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# from utils import Capture, AutoDriver
# from drive import Drive, DriveTest
from uta_racecar.srv import ProcessUsbImageMessage, ProcessUsbImageMessageResponse

import actionlib    

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

def is_printer_active(req):
    result = int(raw_input("Should continue?:   ")) == 1
    return ProcessUsbImageMessageResponse(result)

if __name__ == "__main__":
    # package_init("uta-racecar started")
    rospy.init_node("process_usb_capture")
    rospy.Service("uta_racecar/is_printer_active", ProcessUsbImageMessage, is_printer_active)

    while not rospy.is_shutdown():
        rospy.spin()