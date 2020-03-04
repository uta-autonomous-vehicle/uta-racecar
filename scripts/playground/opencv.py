import time
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from scipy import ndimage
import pdb

def draw_circle_on_frame(img, x,y):
    cv.circle(img, (x,y), 100, (0,0,0), -1)

def show_image(name, img):
    cv.namedWindow(name, cv.WINDOW_NORMAL)
    cv.resizeWindow(name, 1200, 800)

    cv.putText(img, "showing image:" + name, (200,200), cv.FONT_HERSHEY_COMPLEX_SMALL, 2, (255,255,255) )
    cv.imshow(name, img)    

def canny_processing(frame, thres1, thres2):
    # Converts frame to grayscale because we only need the luminance channel for detecting edges - less computationally expensive
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)    
    blur = cv.GaussianBlur(gray, (5, 5), 0) # Applies a 5x5 gaussian blur
    canny = cv.Canny(blur, thres1, thres2)
    
    return canny

'''

while False:
    # ret, frame = cap.read()
    ret, frame = cap.read()
    frame = cv.flip(frame, -1)

    canny = canny_processing(frame)

    # pdb.set_trace()

    # b,g,r = cv.split(frame)
    # frame = cv.merge((r,b,g))

    # print(frame.shape)
    # print(frame[719, 0])

    # if not frame is None and not frame2 is None:
    #     frame = cv.addWeighted(frame, 0.7, frame2, 0.3, 0   )

    y, x, channels = frame.shape

    y_min, y_max = y - 500, y - 500
    x_min, x_max = 0, x

    # cut_frame = frame
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # ret, thresh = cv.threshold(gray, 127, 200, 0)
    # _, contours, hierarchy = cv.findContours(gray, cv.RETR_TREE , cv.CHAIN_APPROX_SIMPLE)

    # cv.drawContours(frame, [contours[4]], 0, (0,255,0), 3)


    # edges = cv.Canny(gray, 200, 300, 3)
    # lines = cv.HoughLinesP(edges, 1, np.pi/180, 80, 30, 10)

    # sample = np.zeros(cut_frame.shape)
    # reshaped_to_original_sample = np.zeros(frame.shape)

    # if not lines is None:
    #     for line in lines:
    #         x1,y1,x2,y2 = line[0]
    #         cv.line(cut_frame,(x1,y1),(x2,y2),(255,255,255), 2)
    #         cv.line(sample,(x1,y1),(x2,y2),(255,255,255), 2)
    
    # frame = cut_frame
    # reshaped_to_original_sample = sample

    # cv.line(frame,(x_min,y_min),(x_max,y_max),(255,0,0), 2)
    

    # show_image('image', frame)
    show_image('image', frame)
    show_image('image', canny)
    # show_image('image1', reshaped_to_original_sample)    
    # show_image('image2', edges)
    # cv.namedWindow('frame', cv.WINDOW_AUTOSIZE)
    
    if cv.waitKey(1) == ord('q'):
        break

    # time.sleep(2)
'''



# cap = None
VIDEO_FILE = 'videos/4.MOV'
cap = cv.VideoCapture(VIDEO_FILE)
def show_canny_with_thres1(thres1, thres2, thresh3, thresh4):
    while cap.isOpened():
        res, frame = cap.read()
        if frame is None:
            break
        
        # frame1 = frame
        frame = cv.flip(frame, -1)
        
        # y, x, z = frame1.shape
        # y = int(y/4)
        # print("frame", y, x, z)
        ret, thresh = cv.threshold(frame, thresh3, thresh4, 0)

        canny = canny_processing(thresh, thres1, thres2)

        # frame2 = np.zeros([frame.shape[0],frame.shape[1]])
        # frame2[y:, :] = canny

        # im = ndimage.rotate(frame, 15, mode='constant')
        # im = ndimage.gaussian_filter(im, 8)
        # sx = ndimage.sobel(im, axis=0, mode='constant')
        # sy = ndimage.sobel(im, axis=1, mode='constant')
        # sob = np.hypot(sx, sy)

        show_image("ORIGINAL", frame)
        show_image("FILTERED", canny)
        # show_image("image3", sob)

        if cv.waitKey(1) == ord('q'):
            break
    
    return False

for i in range(50,150, 20):
    for j in range(100,200,20):
        print("i j", i, j)
        show_canny_with_thres1(10,30, i,j )

cap.release()
# tracking = Tracking(cap)
# tracking.start()

# cap.release()   
# cv.destroyAllWindows()