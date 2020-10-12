import rospy
import math
import time
from datetime import datetime
import os
import cv2
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image, PointCloud2
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

import ros_numpy
from sensor_msgs import point_cloud2

# now = datetime.strftime(datetime.now(), "uta_racecar_%Y-%m-%d-%H:%M:%s")
# PATH_TO_SAVE = "/media/nvidia/samsung_ssd/data/2020/{}".format(now)
SAVE_DATA = True

# right_file = open("/media/nvidia/samsung_ssd/data/2020/left_camera.txt", 'a')

left_seq = 0
right_seq = 0

IMAGE_HEIGHT = rospy.get_param("/uta_racecar/ZED_IMAGE_HEIGHT")
IMAGE_WIDTH = rospy.get_param("/uta_racecar/ZED_IMAGE_WIDTH")
IMAGE_SHAPE = (IMAGE_WIDTH, IMAGE_HEIGHT)

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

    def flush_image_cache(self):
        for i in image_cache:
            self.save_file(self.read_image(i[1]), BaseImageManager.FILE_PATH + "/left_camera/{}.jpg".format(i[0]))

        return
        
    def left_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data, "RGB")
            image = np.asarray(image)

            image_tool = CVTools(image)
            image_tool.add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image
            BaseImageManager.LEFT_CAMERA.write(image)

            logger.info("saving frame {}".format(self.left_seq))

            image = Im.fromarray(image)
            image_path = os.path.join(BaseImageManager.LEFT_CAMERA_DIR, "{}.jpg".format(self.left_seq))
            self.save_file(image,  image_path)

        file_to_write = file_to_write or open(BaseImageManager.FILE_PATH + "/left_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.left_seq))


        if using_standalone:
            file_to_write.write("\n")
            file_to_write.close()

        self.left_seq += 1
        return None

    def right_camera_input(self, data, steering_angle, file_to_write = None, using_standalone = False):
        steering_angle_text = "Angle: {}".format(steering_angle)
        if data.data:
            image = self.read_image(data.data, "RGB")
            image = np.asarray(image)

            image_tool = CVTools(image)
            image_tool.add_text_to_image(steering_angle_text, (100, 100))
            image = image_tool.image
            BaseImageManager.RIGHT_CAMERA.write(image)

            logger.info("saving frame {}".format(self.right_seq))
            
            image_path = os.path.join(BaseImageManager.RIGHT_CAMERA_DIR, "{}.jpg".format(self.left_seq))
            self.save_file(image, image_path)

        file_to_write = file_to_write or open(BaseImageManager.FILE_PATH + "/right_camera.txt", 'a')
        file_to_write.write("{}.jpg ".format(self.right_seq))

        if using_standalone:
            file_to_write.write("\n")
            file_to_write.close()

        self.right_seq += 1
        return None
    
    def point_cloud_input(self, data):
        # pdb.set_trace()
        

        print "recieved a point cloud"

        # points = [[k for k in i] for i in point_cloud2.read_points_list(data, skip_nans=True)]
        # print "length of the points in the cloud %s"%(len(points))
        # BaseImageManager.PC_WRITER.writerow([str(points)])
    
    def ackermann_input(self, data, file_to_write = None, using_standalone = False):
        drive_info = data.drive
        if drive_info:
            file_to_write = file_to_write or open(BaseImageManager.FILE_PATH + "/left_camera.txt", 'a')

            steering_angle = drive_info.steering_angle
            speed = drive_info.speed

            file_to_write.write("{} {}".format(steering_angle, speed))

            if using_standalone:
                file_to_write.write("\n")
                file_to_write.close()

    def sync_camera_steering(self, camera_left, camera_right, steering):
        print "receiving input" 

        self.left_camera_input(camera_left, steering.drive.steering_angle, BaseImageManager.LEFT_CAMERA_TXT)
        self.right_camera_input(camera_right, steering.drive.steering_angle, BaseImageManager.RIGHT_CAMERA_TXT)

        self.ackermann_input(steering, BaseImageManager.LEFT_CAMERA_TXT)
        self.ackermann_input(steering, BaseImageManager.RIGHT_CAMERA_TXT)
        # self.point_cloud_input(point_cloud)

        BaseImageManager.LEFT_CAMERA_TXT.write("\n")
        BaseImageManager.RIGHT_CAMERA_TXT.write("\n")
        # file_to_write.close()
        
        self.seq += 1

        print "main seq {}".format(self.seq)
    
    def shutdown_logged_files(self):
        print "Node shutting down, saving data"
        # print "{} left and {} right images to {}".format(self.left_seq, self.right_seq, BaseImageManager.FILE_PATH)

        BaseImageManager.LEFT_CAMERA_TXT.close()
        BaseImageManager.RIGHT_CAMERA_TXT.close()
        BaseImageManager.AUTONOMOUS_TXT.close()

        BaseImageManager.LEFT_CAMERA.release()
        BaseImageManager.RIGHT_CAMERA.release()

        BaseImageManager.PCT_TEXT.close()

        print("Images and Videos saved to ", )

        # self.convert_images_to_video_seq()

    def register_callbacks_for_saving_data(self):
        # rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.left_camera)
        # rospy.Subscriber("/zed/right/image_rect_color", Image, right_camera)
        
        camera_sub_left = message_filters.Subscriber("/zed/rgb/image_rect_color", Image)
        camera_sub_right = message_filters.Subscriber("/zed/right/image_rect_color", Image)
        angle_sub = message_filters.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped)
        # point_cloud = message_filters.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2)

        rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, self.point_cloud_input)

        ts = message_filters.ApproximateTimeSynchronizer([camera_sub_left, camera_sub_right, angle_sub], 1, 0.1, allow_headerless = True)
        # ts.registerCallback(self.sync_camera_steering)

        rospy.on_shutdown(self.shutdown_logged_files)

        print "registered callbacks for left, right, drive"
        