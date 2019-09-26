
import pyrealsense2 as rs # used to shorten class object to just rs
import cv2
import numpy as np
import matplotlib.pyplot as plt

#create the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file("sample.bag")

#starts streaming
profile = pipeline.start(config)

#skips the first 5 frames
for x in range(5):
    pipeline.wait_for_frames()

"""
stores next frame and retruve depth data out of it
"""
frameset = pipeline.wait_for_frames()
depth_frame = frameset.get_depth_frame()

"""
Stopping the pipeline
"""
pipeline.stop()

"""
Apply color mapping to the depth data
"""
colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

#cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#cv2.imshow('RealSense', colorized_depth)
#cv2.waitKey(1)

"""
decimation filter 
you can change the linear scale factor at 4
"""
decimation = rs.decimation_filter()
decimated_depth = decimation.process(depth_frame)
colorized_depth = np.asanyarray(colorizer.colorize(decimated_depth).get_data())
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', colorized_depth)
cv2.waitKey(1)

"""
decimation.set_option(rs.option.filter_magnitude, 4)
decimated_depth = decimation.process(depth_frame)
colorized_depth = np.asanyarray(colorizer.colorize(decimated_depth).get_data())
plt.imshow(colorized_depth)
"""