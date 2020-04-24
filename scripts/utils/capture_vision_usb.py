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
from path_sense.utils import CVTools, StraightLineOffsetDetector
from path_sense.utils.logger import logger
from image_manager import BaseImageManager

class CaptureSecondaryView(BaseImageManager):
    def __init__(self):
        self.latest_image = None
        self.base_dir = "/racecar/mount/april"
        self.seq = 0
        return
    
    def set_latest_image(self, data):
        self.latest_image = data
    def get_latest_image(self, data):
        return self.latest_image
    
    def callback_image(self, data):
        self.seq += 1

        image = data.data
        image = self.read_image(image, 'RGB', (640, 480))
        if image:
            print "receiving image"
            self.save_file(image, self.base_dir + '/{}.jpg'.format(self.seq))

        # np_image = np.asarray(image)
        # print image
    
    def register_callbacks_for_saving_data(self):
        sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)