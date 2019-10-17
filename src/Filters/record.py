import pyrealsense2 as rs # used to shorten class object to just rs
import cv2
import numpy as np
#from time import clock as timer

pipeline = rs.pipeline()
config = rs.config()

#config settings
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) 
 #set type of stream stream type depth gyro etc res, format, frame 
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_record_to_file('test.bag') #initalize file recording
"""
#can use pause and resume to stop the recoding to the bag without stopping the actual device from streaming 
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.recorder.html?highlight=recorder
"""

pipeline.start(config)
for i in range(0, 1000):
   # start = timer()#starts a timer
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color = frames.get_color_frame()

   # for x in range(200):
    #  rec = rs.recorder

    if not depth_frame:
        continue

    color_image = np.asanyarray(color.get_data())
    colorizer = rs.colorizer(0)
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', colorized_depth)
    cv2.waitKey(1)
    cv2.namedWindow('RealSense1', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense1', color_image)
    cv2.waitKey(1)
    #print (1/ (timer() - start)) #outputs the frames
pipeline.stop()