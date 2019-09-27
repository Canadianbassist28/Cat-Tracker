
import numpy as np
import pyrealsense2 as rs
import cv2

pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("src/Motion/sample.bag")

pipeline.start(cfg)

try:
    while 1:
        #start = timer()

        frames = pipeline.wait_for_frames()
        for frame in frames:

         pr = frame.get_profile()

         #if pr.stream_type() == rs.stream.accel and pr.format() == rs.format.motion_xyz32f:
         #  accelData = frame.as_motion_frame().get_motion_data()
         #  print(accelData)

         if pr.stream_type() == rs.stream.gyro and pr.format() == rs.format.motion_xyz32f:   
           gyroData = frame.as_motion_frame().get_motion_data()
           

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break;
finally:
    pipeline.stop()