
import numpy as np
import pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt
import numpy as np

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
try:
    while 1:

        try:
            frames = pipeline.wait_for_frames()
        except:
            break

        start = timer()
        motion.get_data(frames)
        yaw.append(motion.angle)

        print(1 / (timer() - start))

        #if (cv2.waitKey(1) & 0xFF == ord('q')):
        #	cv2.destroyAllWindows()
        #	break;
finally:
    pipeline.stop()
plt.plot(yaw)
plt.show()

#depth_sensor = profile.get_device().first_depth_sensor()
#preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
#print(str(preset_range.default))
#for i in range(int(preset_range.max)):
#    print(depth_sensor.get_option_value_description(rs.option.visual_preset,i))
#    print(i)