#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This program returns the state of the 3D printer ('Printing' or 'Not Printing')
using the calculated centerpoint position of a pink sticker placed on the print head.
"""

import os
import cv2
import numpy as np
#import matplotlib.pyplot as plt #can use for debugging
#import matplotlib.image as mpimg #can use for debugging

class PrinterState(object):
    def __init__(self):
        self.input_folder = ''
        self.printer_state = ''
        self.images = []
        self.points = []
        self.blobArray = []
        self.counter = int()
        self.wobble_threshold = int()
        self.length = int()

    def get_state():
        self.input_folder = '/home/trav/defect_detection' #set path as nessesary
        self.printer_state = ''
        self.images = []
        self.points = []
        self.blobArray = []
        self.counter = int(0)
        self.wobble_threshold = int(5) #threshold value (in pixels) to account for shaking/wobble/blurring
        self.length = int(0)
       
        try: 
            #store all image paths in folder to array
            for filename in os.listdir(input_folder):
                img = os.path.join(input_folder,filename)
                if img is not None:
                    images.append(img)
            
            length = len(images)
            
            for counter in range(0,length):
                #load image
                current_image = cv2.imread(str(images[counter]))
                
                #convert to hsv color format
                hsv = cv2.cvtColor(current_image, cv2.COLOR_BGR2HSV)
                
                #set color thresholds (HSV upper and lower range values)
                lower_pink = np.uint8([145,85,185])
                upper_pink = np.uint8([170,255,255])
                
                #create a layer mask
                mask = cv2.inRange(hsv,lower_pink, upper_pink)
                
                #imgplot = plt.imshow(mask) #uncomment for debugging
                #plt.show() #uncomment for degugging
                
                #find all contours and store their info
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
                for i,c in enumerate(contours):
                    area = cv2.contourArea(c)
                    blobArray.append(area)
                    
                #sort detected blobs from largest to smallest
                sorted_blobs = sorted(zip(blobArray, contours), key=lambda x: x[0], reverse=True)
            
                #assuming the threshold values were correct and no color noise, the top value should be the pink sticker
                largest_blob = sorted_blobs[0][1]
                
                #determine median point in largest blob
                median_largest_blob_point = np.median(largest_blob,axis=0)
                
                #####uncomment lines below for debugging#######
                #print median values
                #print(median_largest_blob_point)
                #print(median_second_largest_blob_point)
                #print(str(counter))
                ###############################################
                
                #append median value to array to use as a reference point for motion determination
                points.append(median_largest_blob_point)
                print(points)
        except:
                return None
        
        #subtract first image's largest blob median x position value from the second image's to determine motion
        #subject to the wobble threshold value that accounts for shaking/blurring/wobble
        if abs(points[0][0][0] - points[1][0][0]) <= wobble_threshold:
            printer_state = 'PRINTING'
            #print('Not Printing') #uncomment for debugging
        else:
            printer_state = 'NOT PRINTING'
            #print('Printing') #uncomment for debugging
        return printer_state

def main():
    get_state()
    #ret = get_state()
    #print(ret) #print return value to stdout (None - exception occurred, else returns string w/ printer state)
    
if __name__ == "__main__":
    main()
