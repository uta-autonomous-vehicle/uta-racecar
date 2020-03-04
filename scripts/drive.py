#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Drive(object):
    def __init__(self):
        self.started = time.time()

        self.max_speed = 2.0
        self.min_speed = 0.2

        self.current_speed = self.min_speed

        self.acceleration = 0.5
        
        self.safe_front = True
        self.safe_left = True
        self.safe_right = True
        self.safe_rear = True

        self.config = {
            'speed': 1.0,
            'rotation_angle': 0.0,
            'acceleration': 10.0,
            'steering_angle': 0.0,
            'steering_angle_velocity': 0.0,
        }

        self.ack_publisher = rospy.Publisher("/ackermann_cmd_mux/input/default", AckermannDriveStamped, queue_size=1)
    
    
    def get_config(speed, rotation_angle, acceleration = 0.0, steering_angle = 0.0):
        # TODO: add a way to return congigurable config

    def go_forward(secs = 0.0, dist = 0.0):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = self.current_speed

        self.ack_publisher.publish(msg)
        # TODO: add a way to keep moving forward for x seconds or x meters

    def make_turn(angle = 0.0):
        # TODO: add a way to turn at an angle and straighten up after that

    def go_left(angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up

    def go_right(angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up
