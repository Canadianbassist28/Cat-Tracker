
import numpy as np
import pyrealsense2 as rs
import cv2
from realsenseMotion import realsenseMotion

pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.accel)
cfg.enable_stream(rs.stream.gyro)
#cfg.enable_device_from_file("src/Motion/sample.bag")

pipeline.start(cfg)

motion = realsenseMotion()
try:
	while 1:
		#start = timer()

		frames = pipeline.wait_for_frames()

		motion.get_motion_frames(frames)

		print(motion)
			
		if (cv2.waitKey(1) & 0xFF == ord('q')):
			cv2.destroyAllWindows()
			break;
finally:
    pipeline.stop()