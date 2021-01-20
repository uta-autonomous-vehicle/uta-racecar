import rospy
import math
import time
from datetime import datetime
import os
import cv2
from PIL import Image as Im
from sensor_msgs.msg import Joy, Image, PointCloud2, LaserScan
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
from cv_bridge import CvBridge

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

        self.depth_image_seq = 0
        self.scan_seq = 0
        return

    def flush_image_cache(self):
        for i in image_cache:
            self.save_file(self.read_image(i[1]), BaseImageManager.FILE_PATH + "/left_camera/{}.jpg".format(i[0]))

        return
    
    def scan_input(self, data):
        scanner = np.asarray(data.ranges)
        scanner.tofile(os.path.join(BaseImageManager.SCAN_DIR, str(self.seq)))

    def depth_camera_input(self, data):
        if data.data:
            image = self.read_image(data.data, "RGBA", (data.width, data.height))
            image.show()
            
            image = np.asarray(image)

            # not usable output by reading it from cvbridge
            # bridge = CvBridge()
            # cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # self.save_file(cv_image, os.path.join(BaseImageManager.DEPTH_DIR, "cv_{}.jpg".format(self.seq)))

            logger.info("saving frame {}".format(self.depth_image_seq))
            self.save_file(image, os.path.join(BaseImageManager.DEPTH_DIR, "{}.jpg".format(self.seq)))
            self.save_file(image[:,:,3], os.path.join(BaseImageManager.DEPTH_DIR, "depth_{}.jpg".format(self.seq)))

        return None    
    
    def camera_input(self, data, steering_angle = None, file_to_write = None):
        
        if data.data:
            image = self.read_image(data.data, "RGB", (data.width, data.height))

            if steering_angle:
                image_tool = CVTools(np.asarray(image))
                image_tool.add_text_to_image(steering_angle_text, (100, 100))
                logger.info("saving frame {}".format(self.left_seq))

                image = Im.fromarray(image_tool.image)

            image_path = os.path.join(BaseImageManager.LEFT_CAMERA_DIR, "{}.jpg".format(self.seq))
            self.save_file(image, image_path)

        image1 = ros_numpy.numpify(data)
        image1.tofile(os.path.join(BaseImageManager.LEFT_CAMERA_DIR, str(data.header.stamp)))

        return None
    
    def point_cloud_input(self, data):
        print "Recieved a point cloud"
        print "info data ({0}, {1})".format(data.width, data.height)

        # points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        points = ros_numpy.numpify(data)

        file_path = os.path.join(BaseImageManager.PC_DIR, str(self.seq))
        points.tofile(file_path)
    
    def steering_output(self, steering):
        print "steering"
    
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

    def sync_camera_steering(self, rect_color, point_cloud, depth_image, laser_scan):
        print "*"*10
        print "Receiving synced input ", self.seq

        self.camera_input(rect_color)
        self.point_cloud_input(point_cloud)
        self.depth_camera_input(depth_image)
        self.scan_input(laser_scan)

        self.seq += 1

        # steering_angle = steering.drive.steering_angle if steering else 0.0

        # self.camera_input(camera_left, steering_angle, BaseImageManager.LEFT_CAMERA_TXT)
        # self.right_camera_input(camera_right, steering_angle, BaseImageManager.RIGHT_CAMERA_TXT)

        # if steering:
        #     self.ackermann_input(steering, BaseImageManager.LEFT_CAMERA_TXT)
        #     self.ackermann_input(steering, BaseImageManager.RIGHT_CAMERA_TXT)
        #     # self.point_cloud_input(point_cloud)

        # self.seq += 1

        # print "main seq {}".format(self.seq)
    
    def shutdown_logged_files(self):
        print "Node shutting down, saving data to {}".format(BaseImageManager.FILE_PATH)

        BaseImageManager.LEFT_CAMERA_TXT.close()
        BaseImageManager.RIGHT_CAMERA_TXT.close()
        BaseImageManager.AUTONOMOUS_TXT.close()

    def register_callbacks_for_saving_data(self):
        # camera_sub_left = message_filters.Subscriber("/zed/rgb/image_rect_color", Image)
        # camera_sub_right = message_filters.Subscriber("/zed/right/image_rect_color", Image)
        angle_sub = message_filters.Subscriber("/vesc/low_level/ackermann_cmd_mux/output", AckermannDriveStamped)
        
        point_cloud = message_filters.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2)
        # rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, self.point_cloud_input)

        rect_color = message_filters.Subscriber("/zed/rgb/image_rect_color", Image)
        # rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.camera_input)

        depth_image = message_filters.Subscriber("/zed/depth/depth_registered", Image)
        # rospy.Subscriber("/zed/depth/depth_registered", Image, self.depth_camera_input)

        laser_scan = message_filters.Subscriber("/scan", LaserScan)
        # rospy.Subscriber("/scan", LaserScan, self.scan_input)

        ts = message_filters.ApproximateTimeSynchronizer([rect_color, point_cloud, depth_image, laser_scan], 10, 0.5, allow_headerless = True)
        ts.registerCallback(self.sync_camera_steering)

        rospy.on_shutdown(self.shutdown_logged_files)

        print "registered callbacks for left, right, drive"
        