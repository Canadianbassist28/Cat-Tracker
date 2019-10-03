
import pyrealsense2 as rs # used to shorten class object to just rs
import cv2
import numpy as np
import matplotlib.pyplot as plt

#create the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60) #reads form sensor
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_device_from_file("sample.bag") #reads form a file change the the .bag file name

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
    color_frame = frameset.get_color_frame()
    if not depth_frame or not color_frame:
        continue #to the top of the loop

    color_image = np.asanyarray(color_frame.get_data())
    """
    Apply color mapping to the depth data                                                               look at which color maping is beter opencv or realsense
    """
    colorizer = rs.colorizer(0)                                                                         #can have few color format 0 jet 3 blk/wh 2 wh/blk
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

    colorized_depth_org = colorized_depth #creates a copy

    #creates the colored image
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', colorized_depth_org)
    cv2.waitKey(1)


    """
    decimation filter filt
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
    magnitude is num of filter iterations [1-5]
    smooth alpha The Alpha factor in an exponential moving average with Alpha=1 - no filter . Alpha = 0 - infinite filter from [0.25 -1]
    Delta establishes threshold used to preserve edges [1-50]

    can apply hole filling 

    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)
    filtered_depth = spatial.process(depth_frame)                                                           #applys filter of depth frame
    colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())                          #generates colorized imagebased on input depth frames
    cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSenseSpatial', colorized_depth)                                                         #can use cv2.destroy(winname) to close the window and  dealocate 
    cv2.waitKey(1)
    """


    """
    Hole filling 
    fillies missing data setting 0-2
    0: fill from left use value from left neighboring pixel to fill hole
    1: farest_from_arounf usese value from neghiboring pixel thats furthest away
    2: neares_from_around uses value from pixel closest to sensor

    hole_filling = rs.hole_filling_filter()
    filled_depth = hole_filling.process(depth_frame)
    colorized_depth = np.asanyarray(colorizer.colorize(filled_depth).get_data())
    cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSenseSpatial', colorized_depth)                                                         #can use cv2.destroy(winname) to close the window and  dealocate 
    cv2.waitKey(1)
    """


    """
    Applying all the filters
    disparity trasnform allows view in disparity for longer range
    """
    
    decimation = rs.decimation_filter()
    #hole_filling = rs.hole_filling_filter()
    spatial = rs.spatial_filter()
    filter_depth = decimation.process(depth_frame)
    filter_depth = spatial.process(depth_frame)
    #filter_depth = hole_filling.process(depth_frame)
    colorized_depth = np.asanyarray(colorizer.colorize(filter_depth).get_data())
    
    #adds two images 
    images = np.hstack((color_image, colorized_depth ))
    cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSenseSpatial', images)                                                         #can use cv2.destroy(winname) to close the window and  dealocate 
    cv2.waitKey(1)
    #other filters to look threshold depth to desparity temporal
    #temporal disparity

"""
Stopping the pipeline
"""
pipeline.stop()
print("End of code")
