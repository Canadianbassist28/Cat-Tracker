import pyrealsense2 as rs # used to shorten class object to just rs
import cv2
import numpy as np


#create the pipeline
pipeline = rs.pipeline() 
config = rs.config()
#config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

pipeline.start(config)

while True:
    frames = pipeline.wait_for_frames()

    print("practice")