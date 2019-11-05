import cv2
from src.Filters.Camera import realsenseBackbone as backbone
from src.Motion.realsenseMotion import realsenseMotion as Motion
from time import clock as timer
timer()


backbone = backbone()
pipeline = backbone.getpipeline()

x = 0
while x < 10:

    frames = backbone.getFrames() #get the frame from the camera
    timeStamp = frames.get_timestamp() / 1000

    x = .5 * start()
    depth_frame = backbone.getDepthFrame()
    depth_frame = backbone.threshold(depth_frame, 2 + x, 5 + x)
    depth_frame = backbone.disparity()
    depth_frame = backbone.spatial()

    depth_image = backbone.depthImageCV2(depdepth_frame)

    cnts = backbone.contour(depth_image, color_image)

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    res = np.hstack((depthFrame, color_image))
    cv2.imshow('RealSense', res)

    #end the program when the windows is closed
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        cv2.destroyAllWindows()
        break
pipeline.stop()
