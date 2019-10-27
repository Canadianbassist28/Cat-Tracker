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
        #config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)          #set res to 1280 720
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
        #config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        config.enable_device_from_file("sample.bag")    #can do ,false to not repeat
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

    def threePoint(self,x,y):
        """
        Determines the realtive position of the cordinate fro the pesepctive of the cammera.
        @param x coordinate of the pixel
        @param y coordinate of the pixel
        @retrun returns the relative postiton
        """
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        depth_intrins = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        deproject = rs.rs2_deproject_pixel_to_point( depth_intrins,[320, 240], depth_scale)
        return deproject

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
colorizer = rs.colorizer(3)

if __name__ == "__main__":
    while True: #keeps going while it is reciving data
        
        
        frames = backbone.getFrames() #get the frame from the camera
        timeStamp = frames.get_timestamp() / 1000
        motion.get_data(frames, timeStamp) 
        #print(motion.velocity)

        depth_frame = backbone.getDepthFrame(frames)
        color_image = backbone.colorImageCV2(frames)

        disparity = rs.disparity_transform(True)

        # apply the filteras and threshold filter to the image
        #threshold = backbone.threshold(depth_frame, 2, 4)
        depthFrame = backbone.threshold(depth_frame, 2.5, 5)
        depthFrame = backbone.hole(depthFrame)
        depthFrame = disparity.process(depthFrame) 
        depthFrame = backbone.spatial(depthFrame)

        #deci = backbone.decimation(hole)
        depthImage = colorizer.colorize(depth_frame)
        depthImage = np.asanyarray(depthImage.get_data())
        depthFrame = colorizer.colorize(depthFrame)
        depthFrame = np.asanyarray(depthFrame.get_data())


        """
        Apply countour to the image
        In range deterine if th inputed array is within the range of the other two array
        and outputs an array
        """
        low = np.array([110, 110, 110])
        high = np.array([150, 150, 150])
        mask = cv2.inRange(depthFrame, low, high)
        tmp = cv2.bitwise_and(depthImage, color_image, mask = mask)   #calculates the bitwise conjunction of two array
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(color_image, cnts,-1, [0,255,0], 2)


        #get the size of the iamge
        #dimensions = image1.shape
        #print ("Image dimensions: ", dimensions)
        #position = backbone.threePoint(500, 500)
        #print(position)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        res = np.hstack((tmp, color_image))
        cv2.imshow('RealSense', res)
        cv2.waitKey(1)

        #end the program when the windows is closed
        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break;


pipeline.stop()


#work on countor going through the list thats made and processing increments of the pixels and return a 3d point look at getting th contour smother 
#list of pixel pointes