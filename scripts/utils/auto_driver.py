import rospy
import math
import time
from datetime import datetime
import os
import cv2
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from datetime import datetime
import numpy as np
import cv2 as cv

import message_filters
import pdb

from threading import Thread

from drive import Drive
from utils import Capture
from path_sense.utils.cv_tools import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger

now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

class AutoDriverManager(object):
    def __init__(self):
        self.seq_for_autonomy = -1
        self.seq_for_stopping_task = -1

        self._last_stopped_at = 0
    
    def get_last_stopped_at(self):
        return self._last_stopped_at
    def set_last_stopped_at(self, value):
        self._last_stopped_at = value

    def get_seq_for_autonomy(self):
        return self.seq_for_autonomy
    def increment_seq_for_autonomy(self):
        self.seq_for_autonomy += 1
    
    def get_seq_for_stopping_task(self):
        return self.seq_for_stopping_task
    def increment_seq_for_stopping_task(self):
        self.seq_for_stopping_task += 1


class AutoDriver(AutoDriverManager):
    def __init__(self, use_left_camera = False):
        # NOTE: Use left camera for turning outtwards (continous right)
        # NOTE: Use right camera for turning inwards (continous left)

        AutoDriverManager.__init__(self)
        self.use_left_camera = use_left_camera
        self.drive = Drive()
        self.capture = Capture()
        self.save_data = False
    

    def callback_for_autonomy(self, data):
        self.increment_seq_for_autonomy()
        seq = self.get_seq_for_autonomy()
        logger.info("recieved image for autonomy activity at %s", seq)

        image = Im.frombytes("RGB", (1280, 720), data.data)
        image = np.array(image)

        true_offset = 150
        center_offset = true_offset
        if self.use_left_camera:
            center_offset = - true_offset
        
        offset_detector = StraightLineOffsetDetector(image)
        offset_detector.filter_color()
        steering_angle = offset_detector.get_steering_angle(center_offset)
        logger.info("steering angle %s", steering_angle)

        if -0.34 < steering_angle and steering_angle < 0.34:
            self.drive.safety_must_stop_for_blocking_object = False
            # NOTE: TO PREVENT UNDER STEERING
            # if math.fabs(steering_angle) <= 0.1:
            #     self.drive.current_speed = 1.0
            # else:
            #     self.drive.current_speed = 0.5

            self.drive.make_turn(steering_angle)
            
            if self.save_data:
                image = self.capture.read_image(data.data)
                image = np.array(image)

                steering_angle_text = "Angle: {}".format(steering_angle)
                image_tool = CVTools(image)
                image_tool.add_text_to_image(steering_angle_text, (100, 100))
                image = image_tool.image
                if self.use_left_camera:
                    # self.capture.left_camera_video.write(image)
                    self.capture.save_file(Im.fromarray(image), self.capture.file_path + "/left_camera/{}.jpg".format(self.get_seq_for_autonomy()))
                else:
                    # self.capture.right_camera_video.write(image)
                    self.capture.save_file(Im.fromarray(image), self.capture.file_path + "/right_camera/{}.jpg".format(self.get_seq_for_autonomy()))

                self.capture.file_to_write_autonomous.write("{} {}\n".format(self.get_seq_for_autonomy(), steering_angle))
        elif steering_angle == -1:
            pass
            self.drive.safety_must_stop_for_blocking_object = True
        

    def callback_for_halting_activity(self, data):
        self.increment_seq_for_stopping_task()
        seq = self.get_seq_for_stopping_task()
        last_stopped_at = self.get_last_stopped_at()
        
        logger.info("recieved image for halting activity at %s", seq)
        image = Im.frombytes("RGB", (1280, 720), data.data)
        image = np.array(image)

        stopping_mark_detector = StraightLineOffsetDetector(image)
        stopping_mark_detector.filter_color("pink")
        steering_angle_stop = stopping_mark_detector.get_steering_angle(100)

        if steering_angle_stop != -1:
            if (seq - last_stopped_at) < 20:
                return

            logger.info("detected stopping mark at %s", seq)
            self.set_last_stopped_at(seq)

            self.drive.set_driving_around_object_or_halt()
            Thread(target=self.drive.halt, args=(5,0.5)).start()

    def disable_drive(self):
        self.drive.destroy_threads()

        if self.save_data:
            self.capture.shutdown_logged_files()

    def register_callback_for_autonomy(self):
        if self.use_left_camera:
            node_to_listen = '/zed/left/image_rect_color'
        else:
            node_to_listen = '/zed/right/image_rect_color'
        rospy.Subscriber(node_to_listen, Image, self.callback_for_halting_activity)
        rospy.Subscriber(node_to_listen, Image, self.callback_for_autonomy)
        rospy.on_shutdown(self.disable_drive)
        
        self.drive.initiate_threads()
        return
    
    def drive_and_save_data(self):
        self.register_callback_for_autonomy()
        self.save_data = True
        # self.drive.current_speed = 0.5
        self.capture.initiate_setup_to_record_vision()
        return
    
    def drive_autonomous(self):
        self.register_callback_for_autonomy()
        return
