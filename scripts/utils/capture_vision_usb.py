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
from path_sense.utils.cv_tools import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger

class CaptureSecondaryView(object):
    def __init__(self):
        self.latest_image = None
        return
    
    def set_latest_image(self, data):
        self.latest_image = data
    def get_latest_image(self, data):
        return self.latest_image
    
    def callback_image(self, data):
        image = data.data
        self.set_latest_image(image)
        # print image
    
    def register_callbacks_for_saving_data(self):
        sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)