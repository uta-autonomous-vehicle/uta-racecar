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
    def __init__(self):
        pass
    
    def save_file(self, image, file_path_with_name):
        open(file_path_with_name, "w").close()

        if SAVE_DATA:
            image.save(file_path_with_name)

    def read_image(self, data, format = 'RGB', shape = IMAGE_SHAPE):
        # NOTE: streamed data from ZED is in BGR format
        # NOTE: returns a PIL image
        image = Im.frombytes("RGB", shape, data)
        (r,g,b) = image.split()

        if format == 'RGB':
            return Im.merge("RGB", (b,g,r))
        else:
            return image

        # return image