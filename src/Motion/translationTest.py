
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import numpy as np
import cv2
from realsenseMotion import realsenseMotion
from time import clock as timer
timer() 

pipeline = rs.pipeline()
cfg = rs.config()
#cfg.enable_stream(rs.stream.accel)
#cfg.enable_stream(rs.stream.gyro)
cfg.enable_device_from_file("sample.bag", False)

profile = pipeline.start(cfg)
yaw = []

motion = realsenseMotion()
print("starting stream...")
try:
    while 1:

        start = timer()
        try:
            frames = pipeline.wait_for_frames()
            timeStamp = frames.get_timestamp() / 1000
        except:
            break


        motion.get_data(frames, timeStamp)
        cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
    
        yaw.append(motion.linearAccel)

        #print(1 / (timer() - start))

        if (cv2.waitKey(1) & 0xFF == ord('q')):
        	break;
finally:
    pipeline.stop()

cv2.destroyAllWindows()
print("Plot Result")
plt.grid()
plt.plot(yaw)
plt.show()

#depth_sensor = profile.get_device().first_depth_sensor()
#preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
#print(str(preset_range.default))
#for i in range(int(preset_range.max)):
#    print(depth_sensor.get_option_value_description(rs.option.visual_preset,i))
#    print(i)