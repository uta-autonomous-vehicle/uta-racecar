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
from base_image_manager import BaseImageManager

now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

# right_file = open("/media/nvidia/samsung_ssd/data/2020/left_camera.txt", 'a')

left_seq = 0
right_seq = 0

class BaseCapture(object):
    def __init__(self):
        pass

class Capture(BaseCapture, BaseImageManager):
    def __init__(self):
        BaseCapture.__init__(self)
        BaseImageManager.__init__(self)
        self.left_seq = 0
        self.right_seq = 0
        self.seq = 0

        return
    

    def initiate_setup_to_record_vision(self):

        self.begin_date = datetime.strftime(datetime.now(), "UTARACECAR_%Y%m%d.")
        self.begin_time = datetime.strftime(datetime.now(), "%H%M%S")
        
        self.file_path = "/media/nvidia/data/2020/"
        self.file_path += self.begin_date + self.begin_time
        
        os.mkdir(self.file_path)
        # os.mkdir(self.file_path + "images")
        os.mkdir(self.file_path + "/left_camera")
        os.mkdir(self.file_path + "/right_camera")

        open(self.file_path + "/left_camera.txt", "w").close()
        open(self.file_path + "/right_camera.txt", "w").close()
        open(self.file_path + "/drive_autonomous.txt", "w").close()

        self.file_to_write_left = open(self.file_path + "/left_camera.txt", 'a')
        self.file_to_write_right = open(self.file_path + "/right_camera.txt", 'a')
        self.file_to_write_autonomous = open(self.file_path + "/drive_autonomous.txt", 'a')

        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        self.left_camera_video = cv.VideoWriter(self.file_path + '/left_camera.mp4', fourcc, 30.0, (1280,720))
        self.right_camera_video = cv.VideoWriter(self.file_path + '/right_camera.mp4', fourcc, 30.0, (1280,720))

    
    def flush_image_cache(self):
        for i in image_cache:
            self.save_file(self.read_image(i[1]), self.file_path + "/left_camera/{}.jpg".format(i[0]))

        return
        
    def left_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data)
            image = np.asarray(image)

            image_tool = CVTools(image)
            image_tool.add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image
            self.left_camera_video.write(image)

            logger.info("saving frame {}".format(self.left_seq))

            # image = Im.fromarray(image)
            # self.save_file(image, self.file_path + "/left_camera/{}.jpg".format(self.left_seq))

        file_to_write = file_to_write or open(self.file_path + "/left_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.left_seq))


        if using_standalone:
            file_to_write.write("\n")
            file_to_write.close()

        self.left_seq += 1
        return None

    def right_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data)
            image = np.asarray(image)

            image_tool = CVTools(image)
            image_tool.add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image
            self.right_camera_video.write(image)

            logger.info("saving frame {}".format(self.right_seq))
            
            # image = Im.fromarray(image)
            # self.save_file(image, self.file_path + "/right_camera/{}.jpg".format(self.right_seq))

        file_to_write = file_to_write or open(self.file_path + "/right_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.right_seq))

        if using_standalone:
            file_to_write.write("\n")
            file_to_write.close()

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

    def sync_camera_steering(self, camera_left, camera_right, steering):
        self.left_camera_input(camera_left, steering.drive.steering_angle, self.file_to_write_left)
        self.right_camera_input(camera_right, steering.drive.steering_angle, self.file_to_write_right)

        self.ackermann_input(steering, self.file_to_write_left)
        self.ackermann_input(steering, self.file_to_write_right)

        self.file_to_write_left.write("\n")
        self.file_to_write_right.write("\n")
        # file_to_write.close()
        self.seq += 1

        print "main seq {}".format(self.seq)
    
    def shutdown_logged_files(self):
        print "Node shutting down, saving data"
        # print "{} left and {} right images to {}".format(self.left_seq, self.right_seq, self.file_path)

        self.file_to_write_left.close()
        self.file_to_write_right.close()
        self.file_to_write_autonomous.close()

        self.left_camera_video.release()
        self.right_camera_video.release()

        # self.convert_images_to_video_seq()

    def register_callbacks_for_saving_data(self):
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.left_camera)
        # rospy.Subscriber("/zed/right/image_rect_color", Image, right_camera)
        
        camera_sub_left = message_filters.Subscriber("/zed/left/image_rect_color", Image)
        camera_sub_right = message_filters.Subscriber("/zed/right/image_rect_color", Image)
        angle_sub = message_filters.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped)

        ts = message_filters.ApproximateTimeSynchronizer([camera_sub_left, camera_sub_right, angle_sub], 1, 0.1, allow_headerless = True)
        ts.registerCallback(self.sync_camera_steering)

        rospy.on_shutdown(self.shutdown_logged_files)

        print "registered callbacks for left, right, drive"
        