#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program will capture a specified number of images for some specified time
interval between them, then write them to nm output folder.
"""

import cv2
import time

output_folder = '/home/trav/defect_detection' 
camera = 0 #camera source number, replace if nessesary on multi-camera systems

class CaptureImages(object):
    def __init__(self):
        self.num_pics = int()
        self.num_secs = int()
        self.camera = int()
        self.output_folder = ''
        
    def take_pics(int num_pics, int num_secs):
        self.num_pics = num_pics
        self.num_secs = num_secs
        self.camera = int(0) # Replace with proper video capture source on vehicle
        self.output_folder = '/home/trav/defect_detection' # Where to the store image captures
        
        for i in range(1,num_pics):
            try:
                image = cv2.VideoCapture(camera)   
                ret,frame = image.read() # Return a single frame in variable `frame`
                cv2.imwrite(output_folder + '/c' + str(i) + '.jpg',frame) # Write image to file
                image.release() # Clean up
                time.sleep(num_secs) # Delay for 2 seconds between images
            except:
                return None
        return 0
        
def main():
    take_pics()
    #ret = take_pics()
    #print(ret) #print return value to stdout
    
if __name__ == "__main__":
    main()