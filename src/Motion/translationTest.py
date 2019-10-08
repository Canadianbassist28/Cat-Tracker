
import numpy as np
import pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt
import numpy as np

from realsenseMotion import realsenseMotion
from timeit import default_timer as timer


pipeline = rs.pipeline()
cfg = rs.config()
#cfg.enable_stream(rs.stream.accel)
#cfg.enable_stream(rs.stream.gyro)
cfg.enable_device_from_file("src/Motion/sample.bag")

pipeline.start(cfg)

yaw = []

motion = realsenseMotion()
try:
	while len(yaw) < 10000:
		start = timer()

		frames = pipeline.wait_for_frames()

		motion.get_data(frames)
		yaw.append(motion.angle[1])

		#print(1 / (timer() - start))
			 
		#if (cv2.waitKey(1) & 0xFF == ord('q')):
		#	cv2.destroyAllWindows()
		#	break;
finally:
	pipeline.stop()
	plt.plot(yaw)
	plt.show()
