import rospy
import time
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

now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

# right_file = open("/media/nvidia/samsung_ssd/data/2020/left_camera.txt", 'a')

left_seq = 0
right_seq = 0

class BaseCapture(object):
    def __init__(self):
        self.drive = Drive()
        pass

class Capture(BaseCapture):
    def __init__(self):
        super().__init__()
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

        self.file_to_write_left = open(self.file_path + "/left_camera.txt", 'a')
        self.file_to_write_right = open(self.file_path + "/right_camera.txt", 'a')


    def save_file(self, image, file_path_with_name):
        open(file_path_with_name, "w").close()

        if SAVE_DATA:
            image.save(file_path_with_name)
    
    def read_image(self, data, format = 'RGB'):
        # NOTE: streamed data from ZED is in BGR format
        image = Im.frombytes("RGB", (1280, 720), data)
        (r,g,b) = image.split()

        if format == 'RGB':
            return Im.merge("RGB", (b,g,r))
        else:
            return image

        # return image
    
    def flush_image_cache(self):
        for i in image_cache:
            self.save_file(self.read_image(i[1]), self.file_path + "/left_camera/{}.jpg".format(i[0]))

        return
        
    def left_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data)
            
            image_tool = CVTools(image).add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image
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

    def right_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data)

            image_tool = CVTools(image).add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image

            self.save_file(image, self.file_path + "/right_camera/{}.jpg".format(self.right_seq))

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
        self.left_camera_input(camera_left, steering.data.drive.steering_angle, self.file_to_write_left)
        self.right_camera_input(camera_right, steering.data.drive.steering_angle, self.file_to_write_right)

        self.ackermann_input(steering, self.file_to_write_left)
        self.ackermann_input(steering, self.file_to_write_right)

        self.file_to_write_left.write("\n")
        self.file_to_write_right.write("\n")
        # file_to_write.close()
        self.seq += 1

        print "main seq {}".format(self.seq)
    
    def convert_images_to_video_seq(self):
        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        left_camera_video = cv.VideoWriter(self.file_path + '/left_camera.mp4', fourcc, 1,(1280,720))
        right_camera_video = cv.VideoWriter(self.file_path + '/right_camera.mp4', fourcc, 1,(1280,720))

        for i in range(self.left_seq):
            path_name = self.file_path + '/left_camera/{}.jpg'.format(i)
            if os.path.exists(os.path.abspath(path_name)):
                print path_name," exists"
                left_camera_video.write(cv.imread(path_name))
            else:
                print path_name," does not exists"

            path_name = self.file_path + '/right_camera/{}.jpg'.format(i)
            if os.path.exists(os.path.abspath(path_name)):
                right_camera_video.write(cv.imread(path_name))


        left_camera_video.release()
        right_camera_video.release()
    
    def shutdown_logged_files(self):
        print "Node shutting down, saving data"
        print "{} left and {} right images to {}".format(self.left_seq, self.right_seq, self.file_path)

        self.file_to_write_left.close()
        self.file_to_write_right.close()

        # self.convert_images_to_video_seq()

    def register_callbacks_for_saving_data(self):
        # rospy.Subscriber("/zed/left/image_rect_color", Image, self.left_camera)
        # rospy.Subscriber("/zed/right/image_rect_color", Image, right_camera)
        
        camera_sub_left = message_filters.Subscriber("/zed/left/image_rect_color", Image)
        camera_sub_right = message_filters.Subscriber("/zed/right/image_rect_color", Image)
        angle_sub = message_filters.Subscriber("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)

        ts = message_filters.ApproximateTimeSynchronizer([camera_sub_left, camera_sub_right, angle_sub], 10, 0.5, allow_headerless = True)
        ts.registerCallback(self.sync_camera_steering)


        rospy.on_shutdown(self.shutdown_logged_files)

        print "registered callbacks for left, right, drive"
        

class AutoDriver(Capture):
    def __init__(self):
        super().__init__()

    def callback_for_autonomy(self, data):
        image = Im.frombytes("RGB", (1280, 720), data.data)
        image = np.array(image)
        
        steering_angle = StraightLineOffsetDetector(image).get_steering_angle()
        logger.info("steering angle %s", steering_angle)

        self.seq += 1

        if -0.34 < steering_angle and steering_angle < 0.34:
            self.drive.make_turn(steering_angle)
        
    def register_callback_for_autonomy(self):
        rospy.Subscriber("/zed/right/image_rect_color", Image, self.callback_for_autonomy)
        rospy.on_shutdown(self.disable_drive)
        
        self.drive.initiate_threads()
        return
    
    def disable_drive(self):
        self.drive.destroy_threads()
    
    def drive_and_save_data(self):
        self.initiate_setup_to_record_vision()
        self.register_callbacks_for_saving_data()
        self.register_callback_for_autonomy()