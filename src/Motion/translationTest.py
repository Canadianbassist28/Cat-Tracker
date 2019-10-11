
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

playback = profile.get_device().as_playback()

isRecorded = False
if not playback.is_real_time():
    isRecorded = True

motion = realsenseMotion()
print("starting stream...")
try:
    while 1:

        start = timer()
        try:
            frames = pipeline.wait_for_frames()
        except:
            break

        #depth_frame = frames.get_depth_frame()
        #colorizer = rs.colorizer(0) 
        #colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        #cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSenseSpatial', colorized_depth)   

        if isRecorded:
            motion.get_data(frames, playback.get_position())
            print(playback.get_position())
        else:
             motion.get_data(frames)
        yaw.append(motion.velocity)
        print(motion.velocity)
        
        #print(1 / (timer() - start))

        #if (cv2.waitKey(1) & 0xFF == ord('q')):
        #	cv2.destroyAllWindows()
        #	break;
finally:
    pipeline.stop()

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