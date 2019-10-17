import pyrealsense2 as rs
import numpy as np
from time import clock as timer
import cv2


class realsenseBackbone:                                                                                                                            #use sefl to access member funciton and variables 
    """
    Handels the backbone of the camera setting the configs and 
    """
    def __init__(self):
        #self.frames = rs.frames
        self.pipeline = rs.pipeline()
        self.config = self.setConfig() #use
        self.pipeline.start(self.config)
      #  self.frames = self.getFrames()
        
    def setConfig(self):
        #sets the config setting for the cammera
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #config.enable_stream(rs.stream.accel)
        #config.enable_stream(rs.stream.gyro)

        #config.enable_device_from_file("test.bag")    #can do ,false to not repeat
        return config

    def getpipeline(self):
        return self.pipeline

    def getConfig(self):
        return self.config
    
    def getFrames(self):
        #gets the fram from camera containg all the set stream
        frames = self.pipeline.wait_for_frames()
        return frames

    def getDepthFrame(self, frame):
        # input the frame thats collected and returns colorized image ad a array.
        #return depth frames to be able to apply color map.
        depthFrame = frame.get_depth_frame()
        colorized = rs.colorizer()
        colorized_depth = np.asanyarray(colorized.colorize(depthFrame).get_data())
        return colorized_depth

    def getColorFrame(self, frame):
        # input the frame thats collected and returns color frame part.
         colorFrame = frame.get_color_frame()
         colorImage = np.asanyarray(colorFrame.get_data())
         return colorImage



        #for loop in main to continuelsy get frames.


profile = realsenseBackbone()
pipeline = profile.getpipeline()


while True: #keeps going while it is reciving data
        
    frames = profile.getFrames() #get the frame from the camera 
    depth_image = profile.getDepthFrame(frames)
    color_image = profile.getColorFrame(frames)
    #if not depth_image or not color_depth:
      #  continue #to the top of the loop

    # Convert images to numpy arrays
#    depth_image = np.asanyarray(depth_frame.get_data())
 #   color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
  #  depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Stack both images horizontally see both images in same window
    images = np.hstack((color_image, depth_image))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

    if (cv2.waitKey(1) & 0xFF == ord('q')):
        cv2.destroyAllWindows()
        break;


pipeline.stop()

