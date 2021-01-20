#!/usr/bin/python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from utils import Capture, AutoDriver, BaseImageManager
from drive import Drive, DriveTest
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

def test_steering():
    drive_test = DriveTest()
    drive_test.test_steering()
    return None

if __name__ == "__main__":
    package_init("uta-racecar started")
    rospy.init_node("uta_racecar")

    # BaseImageManager()
    # c = Capture()
    # c.register_callbacks_for_saving_data()
    
    driver = AutoDriver(use_left_camera = True)
    driver.drive_autonomous()
    driver.drive_and_save_data()
    
    # driver.initiate_setup_to_record_vision()
    # driver.register_callbacks_for_saving_data()
    # driver.register_callback_for_autonomy()

    while not rospy.is_shutdown():
        rospy.spin()
