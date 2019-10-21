
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


plotData = []
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
    
        plotData.append(motion.velocity)

        #print(1 / (timer() - start))

        if (cv2.waitKey(1) & 0xFF == ord('q')):
        	break;
finally:
    pipeline.stop()

cv2.destroyAllWindows()
print("Plot Result")
plt.grid()
plt.plot(plotData)
plt.show()
