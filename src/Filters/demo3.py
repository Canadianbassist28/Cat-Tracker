import cv2
import numpy as np
from Camera import realsenseBackbone
from src.Motion.realsenseMotion import realsenseMotion as Motion
from time import clock as timer
timer()


backbone = realsenseBackbone()
pipeline = backbone.getpipeline()
with open("test.txt", "w") as outf:
    x = 0
    while x < 10:
    
        frames = backbone.getFrames() #get the frame from the camera
        timeStamp = frames.get_timestamp() / 1000

        x = .5 * timer()
        o_depth_frame = backbone.getDepthFrame(frames)
        depth_frame = backbone.threshold(o_depth_frame, 2 + x, 5 + x)
        depth_frame = backbone.disparity(depth_frame)
        depth_frame = backbone.spatial(depth_frame)

        depth_image = backbone.depthImageCV2(depth_frame)
        color_image = backbone.colorImageCV2(frames)
        cnts = backbone.contour(depth_image, color_image)
        points = backbone.point3DContour(cnts, o_depth_frame)
        outf = backbone.fileOutput(points, outf)


        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        res = np.hstack((depth_image, color_image))
        cv2.imshow('RealSense', res)

        #end the program when the windows is closed
        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break
    pipeline.stop()
