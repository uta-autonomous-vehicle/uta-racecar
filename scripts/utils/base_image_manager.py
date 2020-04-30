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

from drive import Drive
# from path_sense.utils import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger

SAVE_DATA = True

IMAGE_HEIGHT = rospy.get_param("/uta_racecar/ZED_IMAGE_HEIGHT")
IMAGE_WIDTH = rospy.get_param("/uta_racecar/ZED_IMAGE_WIDTH")
IMAGE_SHAPE = (IMAGE_WIDTH, IMAGE_HEIGHT)

class BaseImageManager(object):
    FILE_PATH = ""
    LEFT_CAMERA_DIR = ""
    RIGHT_CAMERA_DIR = ""
    USB_CAMERA_DIR = ""

    LEFT_CAMERA_TXT = ""
    RIGHT_CAMERA_TXT = ""
    AUTONOMOUS_TXT = ""
    USB_CAMERA_TXT= ""

    LEFT_CAMERA = None
    RIGHT_CAMERA = None
    USB_CAMERA = None

    def __init__(self):
        self.initiate_setup_to_record_vision()
        pass

    def create_dir_for_images(self):

        BaseImageManager.LEFT_CAMERA_DIR = os.path.join(BaseImageManager.FILE_PATH, "left_camera")
        BaseImageManager.RIGHT_CAMERA_DIR = os.path.join(BaseImageManager.FILE_PATH, "right_camera")
        BaseImageManager.USB_CAMERA_DIR = os.path.join(BaseImageManager.FILE_PATH, "usb_camera")

        os.mkdir(BaseImageManager.LEFT_CAMERA_DIR)
        os.mkdir(BaseImageManager.RIGHT_CAMERA_DIR)
        os.mkdir(BaseImageManager.USB_CAMERA_DIR)
    
    def create_log_files_for_images(self):
        left_camera_txt = os.path.join(BaseImageManager.FILE_PATH, "left_camera.txt")
        right_camera_txt = os.path.join(BaseImageManager.FILE_PATH, "right_camera.txt")
        usb_camera_txt = os.path.join(BaseImageManager.FILE_PATH, "usb_camera.txt")
        drive_autonomous_txt = os.path.join(BaseImageManager.FILE_PATH, "drive_autonomous.txt")

        open(left_camera_txt, "w").close()
        open(right_camera_txt, "w").close()
        open(usb_camera_txt, "w").close()
        open(drive_autonomous_txt, "w").close()

        BaseImageManager.LEFT_CAMERA_TXT = open(left_camera_txt, 'a')
        BaseImageManager.RIGHT_CAMERA_TXT = open(right_camera_txt, 'a')
        BaseImageManager.USB_CAMERA_TXT = open(usb_camera_txt, 'a')
        BaseImageManager.AUTONOMOUS_TXT = open(drive_autonomous_txt, 'a')
    
    def initiate_setup_to_record_vision(self):
        if BaseImageManager.FILE_PATH:
            logger.warn("Setup already created for this run")
            return

        self.begin_date = datetime.strftime(datetime.now(), "UTARACECAR_%Y%m%d.")
        self.begin_time = datetime.strftime(datetime.now(), "%H%M%S")

        file_path = os.path.abspath("/media/nvidia/data/2020")
        BaseImageManager.FILE_PATH = os.path.join(file_path, self.begin_date + self.begin_time)
        os.mkdir(BaseImageManager.FILE_PATH)

        self.create_dir_for_images()
        self.create_log_files_for_images()

        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        BaseImageManager.LEFT_CAMERA = cv.VideoWriter(BaseImageManager.FILE_PATH + '/left_camera.mp4', fourcc, 30.0, (IMAGE_SHAPE))
        BaseImageManager.RIGHT_CAMERA = cv.VideoWriter(BaseImageManager.FILE_PATH + '/right_camera.mp4', fourcc, 30.0, (IMAGE_SHAPE))
    
    def save_file(self, image, file_path_with_name):
        # image type: PIL.Image
        if type(image) == np.ndarray:
            image = Im.fromarray(image)

        open(file_path_with_name, "w").close()

        if SAVE_DATA:
            image.save(file_path_with_name)
    
    def save_image(self, image, file_path_with_name):
        self.save_file(image, file_path_with_name)

    def read_image(self, data, format = 'RGB', shape = IMAGE_SHAPE):
        # NOTE: streamed data from ZED is in BGR format
        # NOTE: returns a PIL image
        image = Im.frombytes("RGB", shape, data)

        if format == 'RGB':
            (r,g,b) = image.split()
            return Im.merge("RGB", (b,g,r))
        else:
            return image

        # return image