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
        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)          #set res to 1280 720
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #config.enable_stream(rs.stream.accel)
        #config.enable_stream(rs.stream.gyro)

        config.enable_device_from_file("test.bag")    #can do ,false to not repeat
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
        # input the frame thats collected
        #return depth frames
        depthFrame = frame.get_depth_frame()
        return depthFrame

    def getColorFrame(self, frame):
        # inputS the frame thats collected and returns color frame part.
         colorFrame = frame.get_color_frame()
         return colorFrame

    def colorImageCV2(self, frame):
         #puts the color image into a numpy array for cv2 display.
         colorFrame = self.getColorFrame(frame)
         colorImage = np.asanyarray(colorFrame.get_data())
         return colorImage

    def depthImageCV2(self, frame):
        #inputs the frame calls deoth frame to get depth frame.
        #the apply color mappuing to the image into a numpyarray to be displayed in cv2
        depthFrame = self.getDepthFrame(frame)
        colorized = rs.colorizer(0)                                       #can change vaule for color map
        colorized_depth = np.asanyarray(colorized.colorize(depthFrame).get_data())
        return colorized_depth

    def distancePixel(self,depthImage, x, y):
        #send frame and get the frame
        distance = depthImage.get_distance(x, y)
        return distance

        #for loop in main to continuelsy get frames.



profile = realsenseBackbone()
pipeline = profile.getpipeline()


while True: #keeps going while it is reciving data
        
    frames = profile.getFrames() #get the frame from the camera 
    depth_image = profile.depthImageCV2(frames)
    color_image = profile.colorImageCV2(frames)

    #dist = profile.distance(0,0)
    print(profile.distancePixel(frame ,0 ,0))

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

