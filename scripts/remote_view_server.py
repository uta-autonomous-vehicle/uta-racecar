#!/usr/bin/env python

# from flask import Flask, render_template, Response
# from remote_view.camera_view import VideoCamera
from utils import CaptureSecondaryView
import rospy, socket

# app = Flask(__name__)

# @app.route('/')
# def index():
#     return render_template('templates/index.html')

# def gen(camera):
#     while True:
#         frame = camera.get_frame()
#         yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# @app.route('/video_feed')
# def video_feed():
#     return Response(gen(VideoCamera()), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    rospy.init_node("remote_view")

    # host = '10.0.0.103'
    # app.run(host=host+':3000', debug=True)

    usb_camera = CaptureSecondaryView()
    while not rospy.is_shutdown():
        rospy.spin()