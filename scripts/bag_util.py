#!/usr/bin/python

import rosbag
import os

from PIL import Image as Im
from sensor_msgs.msg import Joy, Image

root = '/media/nvidia/data/sept/'
path = ['2020-09-22-17-09-46', '2020-09-28-21-30-37', '2020-09-28-21-32-57', '2020-09-28-15-14-06', '2020-09-28-21-31-20', '2020-09-28-21-33-35' ]

for item in path:
    bag = rosbag.Bag(root + item + '.bag')
    column_names = ['/scan', '/vesc/ackermann_cmd_mux/input/navigation', '']
    # df = pd.DataFrame(columns = column_names)
    for msg in bag.read_messages():
        print msg
    
    # df.to_csv(root + item + '.csv')


for item in path:
    