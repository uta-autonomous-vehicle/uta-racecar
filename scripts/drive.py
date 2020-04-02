#!/usr/bin/env python

import math
import rospy
import time
import datetime

import cv2 as cv

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import numpy as np

from threading import Thread
from Queue import Queue
# import queue
from multiprocessing import Value, Process
from utils.cv_tools import CVTools, StraightLineOffsetDetector

class Drive(object):
    def __init__(self):
        self.started = datetime.datetime.now()

        self.max_speed = 2.0
        self.min_speed = 0.2

        self.max_angle = 0.340000003576

        self.current_speed = 2.0
        self.current_steering_angle = 0.0

        self.acceleration = 1.0
        
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

        self.ack_publisher = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
        self.thread_state = {}
        
        # self.current_steering_thread = Value('d', 0.0)
        self.th = Thread(target = self.keep_going_straight)
        self.must_stop = False
        self.rate = rospy.Rate(500)

        # self.th.join()
    
    def safety_check(self):
        return not self.must_stop

    def initiate_threads(self):
        self.th.start()

    def keep_going_straight(self):
        while self.safety_check() or not rospy.is_shutdown():
            # steering_angle = q.get()
            # if steering_angle:
            #     config = self.get_config(self.current_speed, steering_angle)
            # else:
            config = self.get_config()
            print "going straight with speed {} and angle {}".format(config.drive.speed, config.drive.steering_angle)
            self.ack_publisher.publish(config)

            # self.rate.sleep()

    def get_config(self, speed = None, steering_angle = None, acceleration = None):
        # TODO: add a way to return congigurable config

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering_angle or self.current_steering_angle
        msg.drive.acceleration = acceleration or 0.0
        msg.drive.speed = speed or self.current_speed

        return msg

        # self.ack_publisher.publish(msg

    def go_forward(self, secs = 0.0, dist = 0.0, angle = 0.0):
        start = datetime.datetime.now()
        end = datetime.datetime.now()

        while (datetime.datetime.now() - start).seconds < secs:
            self.current_speed = 2.0
            # config = self.get_config(self.current_speed, angle)

            # self.ack_publisher.publish(config)
            end = datetime.datetime.now()
            self.rate.sleep()
        
        self.current_speed = 0.0
        # self.must_stop = True
        # TODO: add a way to keep moving forward for x seconds or x meters
    
    def go_back(self, secs = 0.0, dist = 0.0):
        if secs:
            start = datetime.datetime.now()
            end = datetime.datetime.now()

            while (datetime.datetime.now() - start).seconds < secs:
                self.current_speed = -2.0
                end = datetime.datetime.now()
            # TODO: add a way to keep moving forward for x seconds or x meters

            self.current_speed = 0.0

        self.current_speed = 0.0

    def make_turn(self, angle = 0.0):
        # TODO: add a way to turn at an angle and straighten up after that

        self.current_steering_angle = angle
        return True

    def go_left(self, angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up
        start = datetime.datetime.now()

        while (datetime.datetime.now() - start).seconds < 3.4:
            self.current_speed = 2.0
            self.current_steering_angle = self.max_angle
            
        self.current_speed = 0.0

        return

    def go_right(self, angle = 0.0):
        # TODO: add a way to make a 90 turn and straighten up
        start = datetime.datetime.now()

        while (datetime.datetime.now() - start).seconds < 3.4:
            self.current_speed = 2.0
            self.current_steering_angle = - self.max_angle
            
        self.current_speed = 0.0

        return

    
    def test_steering(self):
        rate = rospy.Rate(10)
        self.current_speed = 0.0
        for i in np.linspace(-0.34, 0.34, num = 100):
            for _ in range(1000):
                self.make_turn(i)
            rate.sleep()