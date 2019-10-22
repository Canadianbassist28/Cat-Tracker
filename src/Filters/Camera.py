import pyrealsense2 as rs
import numpy as np
from time import clock as timer
import cv2
from src.Motion.realsenseMotion import realsenseMotion


class realsenseBackbone():                                                                                                                            #use sefl to access member funciton and variables 
    """
    Handels the backbone of the camera setting the configs and 
    """
    def __init__(self):
        #self.frames = rs.frames
        self.pipeline = rs.pipeline()
        self.config = self.setConfig() #use
        self.profile = self.pipeline.start(self.config)
      #  self.frames = self.getFrames()
        
    def setConfig(self):
        #sets the config setting for the cammera
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)          #set res to 1280 720
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        #config.enable_device_from_file("sample.bag", False)    #can do ,false to not repeat
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

    def depthImageCV2(self, depthFrame):
        #inputs the depth frame
        #then apply color mappuing to the image into a numpyarray to be displayed in cv2
        colorized = rs.colorizer(0)                                       #can change vaule for color map
        colorized_depth = np.asanyarray(colorized.colorize(depthFrame).get_data())
        return colorized_depth

    def distancePixel(self,depthFrame, x, y):
        #takes in a depth frame form the camera and cordinates of the pixel 
        #that the depth is wanted. returns the depth in meters
        distance = depthFrame.get_distance(x, y)
        return distance

    """
    Filters settings.
    must call depthimageCv2 to display any filters
    To get decimation to work must apply after hole filing not before.
    """
    def decimation(self, frame):
        #apply decimation filter to the frame that is sent in. Downscaling
        #must call depthImageCV2 to display

        decimation = rs.decimation_filter()
        decimation.set_option(rs.option.filter_magnitude, 2)
        decimated_depth = decimation.process(frame)
        return decimated_depth

    def spatial(self, frame):
        #apply spatial filtering to the frame sent in 
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 5)
        spatial.set_option(rs.option.filter_smooth_alpha, .25)
        spatial.set_option(rs.option.filter_smooth_delta, 50)
        #spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(frame)
        return filtered_depth

    def hole(self, frame):
        hole_filling = rs.hole_filling_filter()
        holeFilling = hole_filling.process(depth_frame)
        return holeFilling

    def threshold(self, frame, minDistance, maxDistance):
        threshold_filter = rs.threshold_filter(minDistance, maxDistance)
        return threshold_filter.process(frame)






backbone = realsenseBackbone()
motion = realsenseMotion()
pipeline = backbone.getpipeline()

if __name__ == "__main__":
    while True: #keeps going while it is reciving data
        
        
        frames = backbone.getFrames() #get the frame from the camera
        timeStamp = frames.get_timestamp() / 1000
        motion.get_data(frames, timeStamp) 
        print(motion.velocity)

        depth_frame = backbone.getDepthFrame(frames)
        depth_image = backbone.depthImageCV2(depth_frame)
        color_image = backbone.colorImageCV2(frames)

        # Stack both images horizontally see both images in same window
        deci1 = backbone.spatial(depth_frame)
        #hole = backbone.hole(deci1)
        #deci = backbone.decimation(deci1)
        image1 = backbone.depthImageCV2(deci)
        #image2 = backbone.depthImageCV2(deci1)
        #images = np.hstack((image1, image2))

        #get the depth of at a pixel
        depth_sensor = backbone.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        depth_intrins = backbone.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        print(rs.rs2_deproject_pixel_to_point( depth_intrins,[320, 240], depth_scale))

        #get the size of the iamge
        #dimensions = image1.shape
        #print ("Image dimensions: ", dimensions)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', image1 )
        cv2.waitKey(1)

        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break;


pipeline.stop()

