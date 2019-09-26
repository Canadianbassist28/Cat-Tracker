
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
#for x in range(5):
#   pipeline.wait_for_frames()

while True: # keeps going through the image  
    """
    stores next frame and retrives depth data out of it
    """
    frameset = pipeline.wait_for_frames()
    depth_frame = frameset.get_depth_frame()

    """
    Apply color mapping to the depth data                                                               look at which color maping is beter opencv or realsense
    """
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', colorized_depth)
    cv2.waitKey(1)

    """
    decimation filter 
    reduces depth scene complexity or downsampaling sampaling signal at a lower rate
    makes the image shown smaller 
    Has an adjustable linear scale factor at filter_magnitude range 2-8 default is 2

    decimation = rs.decimation_filter()
    decimation.set_option(rs.option.filter_magnitude, 3)
    decimated_depth = decimation.process(depth_frame)
    colorized_depth = np.asanyarray(colorizer.colorize(decimated_depth).get_data())
    cv2.namedWindow('RealSenseDecimation', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSenseDecimation ', colorized_depth)                                                         #can use cv2.destroy(winname) to close the window and  dealocate 
    cv2.waitKey(1)
    """

    """
    Spatial filtering
    smoothes the recorded images
    changing smooth_alpha and delta changes how much the filter is applyed to the images
    """
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    filtered_depth = spatial.process(depth_frame)       #applys filter of depth frame
    colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())  #generates colorized imagebased on input depth frames
    cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSenseSpatial', colorized_depth)                                                         #can use cv2.destroy(winname) to close the window and  dealocate 
    cv2.waitKey(1)

"""
Stopping the pipeline
"""
pipeline.stop()
print("End of code")
