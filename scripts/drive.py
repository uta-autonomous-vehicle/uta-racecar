#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import numpy as np

from threading import Thread

class Drive(object):
    def __init__(self):
        self.started = time.time()

        self.max_speed = 2.0
        self.min_speed = 0.2

        self.current_speed = 2.0

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
        self.thread_state = {}
    
    
    def get_config(self, speed = 0.0, steering_angle = 0.0, acceleration = 0.0):
        # TODO: add a way to return congigurable config

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering_angle or 0.0
        msg.drive.acceleration = acceleration or 0.0
        msg.drive.speed = speed or self.current_speed

        return msg

        # self.ack_publisher.publish(msg

    def go_forward(self, secs = 0.0, dist = 0.0):
        start = time.time()

        while (end - start) < secs:
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = 0.0
            msg.drive.speed = self.current_speed

            self.ack_publisher.publish(msg)
            end = time.time()
        # TODO: add a way to keep moving forward for x seconds or x meters

    def make_turn(self, angle = 0.0):
        # TODO: add a way to turn at an angle and straighten up after that

        print "making a turn at an angle ", angle
        config = self.get_config(None, angle)
        self.ack_publisher.publish(config)

    def go_left(self, angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up
        return

    def go_right(self, angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up
        return

    
    def test_steering(self):
        rate = rospy.Rate(10)
        self.current_speed = 0.0
        for i in np.linspace(-0.34, 0.34, num = 100):
            for _ in range(1000):
                self.make_turn(i)
            rate.sleep()