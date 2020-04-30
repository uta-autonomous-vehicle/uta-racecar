import socket
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

import os
import message_filters
import pdb

from drive import Drive
from path_sense.utils import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger
from base_image_manager import BaseImageManager

import roslaunch


# PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)

class CaptureSecondaryView(BaseImageManager):
    def __init__(self):
        BaseImageManager.__init__(self)
        self.begin_date = datetime.strftime(datetime.now(), "UTARACECAR_%Y%m%d.")
        self.begin_time = datetime.strftime(datetime.now(), "%H%M%S")

        self.latest_image = None
        # self.base_dir = PATH_TO_SAVE

        # os.mkdir(self.base_dir)

        self.seq = 0
        self.video_seq = 0
        self.register_callbacks_for_saving_data()
        return
    
    def set_latest_image(self, data):
        self.latest_image = data
    def get_latest_image(self, data):
        return self.latest_image
    
    def extract_image(self, data, rgb_format = "RAW"):
        image_raw = data
        image_height = rospy.get_param("/usb_cam/image_height")
        image_width = rospy.get_param("/usb_cam/image_width")
        
        image = self.read_image(image_raw, rgb_format, (image_width, image_height))
        logger.debug("extracting image from raw data")
        return image
    
    def callback_image(self, data):
        logger.debug("request to record and image from USB received at callback")
        self.seq += 1

        image_raw = data.data
        image = self.extract_image(image_raw)
        
        if image:
            image_path = os.path.join(BaseImageManager.USB_CAMERA_DIR, "{}.jpg".format(self.seq))
            self.save_file(image, image_path)
    
    def callback_record_video(self, data):
        self.video_seq += 1
        logger.debug("request to record video received %s", self.video_seq)
        if not BaseImageManager.USB_CAMERA or not data:
            return

        logger.debug("writing video")        
        image_raw = data.data
        image = self.extract_image(image_raw, "RGB")
        if image:
            logger.debug("extracted frame for video")
            image = np.asarray(image)
            BaseImageManager.USB_CAMERA.write(image)

    def record_short_video(self, duration = 10.0):
        # duration type:int in seconds
        started = datetime.now()
        
        video_file_name = "{}.mp4".format(datetime.strftime(started, "%H%M%S"))
        video_path = os.path.join(BaseImageManager.USB_CAMERA_DIR, video_file_name )
        framerate = rospy.get_param("/usb_cam/framerate")

        image_height = rospy.get_param("/usb_cam/image_height")
        image_width = rospy.get_param("/usb_cam/image_width")
        image_shape = (image_width, image_height)

        logger.info("setting up recorder %s", framerate)
        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        BaseImageManager.USB_CAMERA = cv.VideoWriter(video_path, fourcc, float(framerate), (image_shape))
        logger.info("set recorder")
        
        self.video_seq = 0
        self.usb_image_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_record_video)
        
        while (datetime.now() - started).seconds < duration:
            logger.info("waiting for duration")
            time.sleep(1)
            pass
        
        self.usb_image_subscriber.unregister()
        time.sleep(2)
        BaseImageManager.USB_CAMERA.release()
    
    def register_callbacks_for_saving_data(self):
        # self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)
        return True