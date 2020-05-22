import rospy
import math
import time
from datetime import datetime
import os
import cv2
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import std_msgs.msg as std_msgs
from datetime import datetime
import numpy as np
import cv2 as cv

import message_filters
import pdb

from threading import Thread

from drive import Drive
from capture_vision_steering import Capture
from capture_vision_usb import CaptureSecondaryView
from path_sense.utils.cv_tools import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger
from uta_racecar.srv import ProcessUsbImageMessage, ProcessUsbImageMessageResponse
from base_image_manager import BaseImageManager


# now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
# PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

IMAGE_HEIGHT = rospy.get_param("/uta_racecar/ZED_IMAGE_HEIGHT")
IMAGE_WIDTH = rospy.get_param("/uta_racecar/ZED_IMAGE_WIDTH")

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
    HALTING_MARK = "halting_mark"
    TRACKER = {
        HALTING_MARK: 0
    }

    def __init__(self, use_left_camera = False):
        # NOTE: Use left camera for turning outtwards (continous right)
        # NOTE: Use right camera for turning inwards (continous left)

        AutoDriverManager.__init__(self)
        self.use_left_camera = use_left_camera
        self.drive = Drive()
        self.seconday_view = CaptureSecondaryView()
        self.capture = Capture()
        self.save_data = False

    def callback_for_autonomy(self, data):    
        self.increment_seq_for_autonomy()
        seq = self.get_seq_for_autonomy()
        logger.info("recieved image for autonomy activity at %s", seq)

        image = Im.frombytes("RGB", (IMAGE_WIDTH, IMAGE_HEIGHT), data.data)
        # image_height = rospy.get_param("/usb_cam/image_height")
        # image_width = rospy.get_param("/usb_cam/image_width")

        # image = self.capture.read_image(data.data, 'RGB', (IMAGE_WIDTH, IMAGE_HEIGHT))
        image_array = np.array(image)

        true_offset = 200
        center_offset = true_offset
        if self.use_left_camera:
            center_offset = -1 * true_offset
        
        offset_detector = StraightLineOffsetDetector(image_array)
        offset_detector.filter_color()
        steering_angle = offset_detector.get_steering_angle(center_offset)
        logger.info("steering angle %s", steering_angle)

        if -0.34 < steering_angle and steering_angle < 0.34:
            logger.info("path detected. to steer in %s angle", steering_angle)
            self.drive.safety_must_stop_for_no_path = False
            # NOTE: TO PREVENT UNDER STEERING
            if math.fabs(steering_angle) <= 0.1:
                self.drive.current_speed = self.drive.max_speed
            else:
                self.drive.current_speed = self.drive.max_speed * 0.7

            self.drive.make_turn(steering_angle)
            
        elif steering_angle == -1:
            # pass
            self.drive.safety_must_stop_for_no_path = True
        
        if self.save_data:
            # image = self.capture.read_image(data.data, "RAW")
            # image = np.array(image)

            steering_angle_text = "Angle: {}".format(steering_angle)
            # image_tool = CVTools(image)
            # image_tool.add_text_to_image(steering_angle_text, (100, 100))
            # image = image_tool.image

            image_path = os.path.join(BaseImageManager.AUTONOMOUS_DIR, "{}.jpg".format(self.get_seq_for_autonomy()))
            self.capture.save_file(image, image_path)
    
    def take_input(self):
        while True:
            ip = raw_input("Enter: 1:Continue 2:Halt 3:Shutdown 4:Go Around")
            if ip == "1":
                return True
            if ip == "2":
                self.drive.halt(5)
            if ip == "3":
                rospy.signal_shutdown("Stopping manually")
            if ip == "4":
                self.drive.go_right_circle()
                

    def callback_for_halting_activity(self, data):
        self.increment_seq_for_stopping_task()
        seq = self.get_seq_for_stopping_task()
        last_stopped_at = self.get_last_stopped_at()
        
        logger.info("recieved image for halting activity at %s", seq)

        image = Im.frombytes("RGB", (IMAGE_WIDTH, IMAGE_HEIGHT), data.data)
        # image_height = rospy.get_param("/usb_cam/image_height")
        # image_width = rospy.get_param("/usb_cam/image_width")
        
        # image = self.capture.read_image(data.data, 'RGB', (IMAGE_WIDTH, IMAGE_HEIGHT))
        image_array = np.array(image)

        stopping_mark_detector = StraightLineOffsetDetector(image_array)
        stopping_mark_detector.filter_color("pink")
        steering_angle_stop = stopping_mark_detector.get_steering_angle(100)

        if steering_angle_stop != -1:
            if (seq - last_stopped_at) < 100:
                return
            
            if AutoDriver.TRACKER[AutoDriver.HALTING_MARK] == 0:
                AutoDriver.TRACKER[AutoDriver.HALTING_MARK] = 1

        else:
            if AutoDriver.TRACKER[AutoDriver.HALTING_MARK] == 1:
                logger.info("detected stopping mark at %s", seq)
                self.set_last_stopped_at(seq)
                self.drive.set_driving_around_object_or_halt()
                
                time.sleep(.5)
                self.drive.halt(2)
                
                # s = rospy.ServiceProxy("uta_racecar/is_printer_active", ProcessUsbImageMessage)
                # if s(1).result:
                #     pass
                # else:
                #     rospy.signal_shutdown("Stopping manually")

                # self.take_input()
                CaptureSecondaryView().record_short_video()
                # self.drive.go_sright_circle()

                self.drive.current_speed = self.drive.max_speed
                self.drive.reset_driving_around_object_or_halt()
                # Thread(target=self.drive.halt, args=(5,0.5)).start()
                AutoDriver.TRACKER[AutoDriver.HALTING_MARK] = 0

    def callback_for_safety(self, data):
        logger.info("received callback_for_safety: %s", data.data)
        self.drive.safety_must_stop_for_blocking_object = not data.data

    def disable_drive(self):
        self.drive.destroy_threads()

        if self.save_data:
            self.capture.shutdown_logged_files()

    def register_callback_for_autonomy(self):
        if self.use_left_camera:
            node_to_listen = '/zed/left/image_rect_color'
        else:
            node_to_listen = '/zed/right/image_rect_color'
        
        # node_to_listen = '/zed/left/image_rect_color'
        rospy.Subscriber(node_to_listen, Image, self.callback_for_halting_activity)
        rospy.Subscriber(node_to_listen, Image, self.callback_for_autonomy)
        rospy.Subscriber("/safety_node/safety", std_msgs.Bool, self.callback_for_safety)
        

        # self.cv_publisher = rospy.Publisher('/uta_racecar/autonomous_vision', Image, 10)
        rospy.on_shutdown(self.disable_drive)
        
        self.drive.initiate_threads()
        return
    
    def drive_and_save_data(self):
        self.register_callback_for_autonomy()
        self.save_data = True
        # self.capture.register_callbacks_for_saving_data()
        # self.drive.current_speed = 0.5
        # self.capture.initiate_setup_to_record_vision()
        return
    
    def drive_autonomous(self):
        self.register_callback_for_autonomy()
        return