import rospy
import time
import os
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from datetime import datetime
import numpy as np

import pdb

now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)

# right_file = open("/media/nvidia/samsung_ssd/data/2020/left_camera.txt", 'a')

left_seq = 0
right_seq = 0

class BaseCapture(object):
    def __init__(self):
        pass

class Capture(BaseCapture):
    def __init__(self):
        self.left_seq = 0
        self.right_seq = 0
        self.begin_date = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d.")
        self.begin_time = datetime.strftime(datetime.now(), "%H:%M:%S")
        
        self.file_path = "/media/nvidia/samsung_ssd/data/2020/"
        self.file_path += self.begin_date + self.begin_time
        
        os.mkdir(self.file_path)
        # os.mkdir(self.file_path + "images")
        os.mkdir(self.file_path + "/left_camera")
        os.mkdir(self.file_path + "/right_camera")

        open(self.file_path + "left_camera.txt", "w").close()
        open(self.file_path + "right_camera.txt", "w").close()

    def save_file(self, image, file_path_with_name):
        open(file_path_with_name, "w").close()

        image.save(file_path_with_name)
    
    def read_image(self, data):
        # data type: string
        image = Im.frombytes("RGB", (1280, 720), data)
        (r,g,b) = image.split()

        image = Im.merge("RBG", (b,g,r))
        return image
        
    def left_camera(self, data):
        if self.left_seq == 10:
            pdb.set_trace()

        if data.data:
            image = self.read_image(data.data)
            # im = np.fromstring(data.data)
            # im = im.reshape((720, 480))
            # print(im.shape)
            self.save_file(image, self.file_path + "/left_camera/{}.jpg".format(self.left_seq))
        f = open(self.file_path + "/left_camera.txt", 'a')
        f.write("{}.jpg {}\n".format(self.left_seq, ""))
        f.close()

        self.left_seq += 1
        return None

    def right_camera(self, data):
        if data.data:
            image = self.read_image(data.data)
            self.save_file(image, self.file_path + "/right_camera/{}.jpg".format(self.right_seq))
        
        f = open(self.file_path + "/left_camera.txt", 'a')
        f.write("{}.jpg {}\n".format(self.right_seq, ""))
        f.close()

        self.right_seq += 1
        return None

    def listener(self):
        rospy.Subscriber("/zed/left/image_rect_color", Image, self.left_camera)
        # rospy.Subscriber("/zed/right/image_rect_color", Image, right_camera)
        return