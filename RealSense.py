import pyrealsense2 as rs
import cv2
import numpy as np
from timeit import default_timer as timer


pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

pipeline.start(config)

#align = rs.align(rs.stream.color)

try:
    while 1:
        start = timer()


        #depth_frame = aligned_frames.get_depth_frame()
        #color_frame = aligned_frames.get_depth_frame()

        #depth_image = np.asanyarray(depth_frame.get_data())
        #color_image = np.asanyarray(color_frame.get_data())

        #depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_JET)

        #color_image = np.expand_dims(color_image, axis = 3)

        #end = timer()
        #FPS = 1.0 / (end - start)
        #cv2.imshow("Depth", depth_color)

        frames = pipeline.wait_for_frames()
        for frame in frames:

          pr = frame.get_profile()

          if pr.stream_type() == rs.stream.accel and pr.format() == rs.format.motion_xyz32f:
               data = frame.as_motion_frame().get_motion_data()
               print(type(data))
               #print(pr.stream_type(), data)



        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break;
finally:
    pipeline.stop()
