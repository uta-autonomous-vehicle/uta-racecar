#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import actionlib    
from utils.capture_vision_steering import Capture
print "............"
def joy_callback(msg):
    # TODO: subscribed to /joy
    pass

# change_time = rospy.Time.now()

def get_time_passed(started = None):

    if not started:
        return 0
    
    return time.time() - started

# if __name__ == "__main__":
#     print "starting uta-racecar node"
#     rospy.init_node("uta-racecar")
    
#     max_speed = rospy.get_param("~max_speed", 1)
#     max_steering = rospy.get_param("~max_steering", 1.04)
#     rospy.Subscriber('/joy', Joy, joy_callback)

#     ack_publisher = rospy.Publisher("/ackermann_cmd_mux/input/default", AckermannDriveStamped, queue_size=1)

#     n = 0

#     started = time.time()
#     # client = actionlib.SimpleActionClient('/ackermann_cmd_mux/input/default', AckermannDriveStamped)
    
#     while True:
#         # print get_time_passed(started)
#         if get_time_passed(started) > 5:
#             break

#         ack_header = Header()
#         # ack_header.seq = n
#         ack_header.stamp.secs = get_time_passed()
#         n += 1
    
#         ack_drive = AckermannDrive()
#         # ack_drive.steering_angle = -0.5
#         # ack_drive.steering_angle_velocity = 15.0
#         ack_drive.speed = 0.4
#         # ack_drive.acceleration = 10.0
#         # ack_drive.serialize = 
    
#         sample_msg = AckermannDriveStamped()
#         sample_msg.drive = ack_drive
#         # sample_msg.header = ack_header
        
#         ack_publisher.publish(sample_msg)

#         # print(sample_msg)
#         # break
#         # time.sleep(0.2)

#         # client.send_goal(ack_drive)

#     print "exits"
#     # while not rospy.is_shutdown():
#     #     rospy.spin()

if __name__ == "__main__":
    rospy.init_node("uta_racecar")

    capture = Capture()
    capture.listener()
    
    while not rospy.is_shutdown():
        rospy.spin()
