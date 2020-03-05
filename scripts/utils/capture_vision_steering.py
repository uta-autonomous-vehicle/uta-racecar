import rospy
import time
import os
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from datetime import datetime
import numpy as np

import message_filters
import pdb

now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

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
        self.seq = 0

        self.begin_date = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d.")
        self.begin_time = datetime.strftime(datetime.now(), "%H:%M:%S")
        
        self.file_path = "/media/nvidia/samsung_ssd/data/2020/"
        self.file_path += self.begin_date + self.begin_time
        
        os.mkdir(self.file_path)
        # os.mkdir(self.file_path + "images")
        os.mkdir(self.file_path + "/left_camera")
        os.mkdir(self.file_path + "/right_camera")

        open(self.file_path + "/left_camera.txt", "w").close()
        open(self.file_path + "/right_camera.txt", "w").close()

    def save_file(self, image, file_path_with_name):
        open(file_path_with_name, "w").close()

        if SAVE_DATA:
            image.save(file_path_with_name)
    
    def read_image(self, data):
        # data type: string
        image = Im.frombytes("RGB", (1280, 720), data)
        (r,g,b) = image.split()

        image = Im.merge("RGB", (b,g,r))
        return image
        
    def left_camera(self, data, file_to_write = None, using_standalone = False):
        if data.data:
            image = self.read_image(data.data)
            # im = np.fromstring(data.data)
            # im = im.reshape((720, 480))
            # print(im.shape)
            self.save_file(image, self.file_path + "/left_camera/{}.jpg".format(self.left_seq))
        file_to_write = file_to_write or open(self.file_path + "/left_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.left_seq))

        if using_standalone:
            file_to_write.write("\n")
            file_to_write.close()

        self.left_seq += 1
        return None

    def right_camera(self, data, file_to_write = None, using_standalone = False):
        if data.data:
            image = self.read_image(data.data)
            self.save_file(image, self.file_path + "/right_camera/{}.jpg".format(self.right_seq))
        
        file_to_write = file_to_write or open(self.file_path + "/right_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.right_seq))
        # file_to_write.close()

        self.right_seq += 1
        return None
    
    def ackermann_input(self, data, file_to_write = None, using_standalone = False):
        drive_info = data.drive
        if drive_info:
            file_to_write = file_to_write or open(self.file_path + "/left_camera.txt", 'a')

            steering_angle = drive_info.steering_angle
            speed = drive_info.speed

            file_to_write.write("{} {}".format(steering_angle, speed))

            if using_standalone:
                file_to_write.write("\n")
                file_to_write.close()

    def sync_camera_steering(self, camera, steering):
        file_to_write = open(self.file_path + "/left_camera.txt", 'a')
        self.left_camera(camera, file_to_write)
        self.ackermann_input(steering, file_to_write)

        file_to_write.write("\n")
        file_to_write.close()
        self.seq += 1

        print "main seq {}".format(self.seq)

    def listener(self):
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.left_camera)
        # rospy.Subscriber("/zed/right/image_rect_color", Image, right_camera)
        
        camera_sub = message_filters.Subscriber("/zed/left/image_rect_color", Image)
        angle_sub = message_filters.Subscriber("/ackermann_cmd_mux/input/teleop", AckermannDriveStamped)

        ts = message_filters.ApproximateTimeSynchronizer([camera_sub, angle_sub], 10, 0.5, allow_headerless = True)
        ts.registerCallback(self.sync_camera_steering)

        print "registered callbacks"
        
        return