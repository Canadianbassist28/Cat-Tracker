
import pyrealsense2 as rs
import cv2
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.steam.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

try:
    while 1:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        depth_image = np.asanyarray(depth_frame.get_data())

        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)

        cv2.imshow("Image view", depth_color)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
                    break;
finally:
    pipeline.stop()
