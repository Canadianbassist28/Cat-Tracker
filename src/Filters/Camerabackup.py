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
        #config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
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
        colorized = rs.colorizer(3)                                       #can change vaule for color map
        colorized_depth = np.asanyarray(colorized.colorize(depthFrame).get_data())
        return colorized_depth

    """
    distance and 3d point realtive to the cammera functions
    """

    def distancePixel(self,depthFrame, x, y):
        #takes in a depth frame form the camera and cordinates of the pixel 
        #from where the depth is wanted. returns the depth in meters
        distance = depthFrame.get_distance(x, y)
        return distance

    def threePoint(self,depth_frame,x,y):
        """
        Determines the realtive 3D position of the cordinate from the pesepctive of the cammera.
        @param x coordinate of the pixel
        @param y coordinate of the pixel
        @param depth_frame retrived by the cammera to get the distance at the pixel
        @retrun returns the relative postiton
        """
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        depth_intrins = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        depth = self.distancePixel(depth_frame,x,y)
        deproject = rs.rs2_deproject_pixel_to_point( depth_intrins,[320, 240], depth)
        return deproject

    def point3DContour(self, cnts):
        """
        Determines the 3d point of portions of th econtours no set point that is being loked at.
        @param cnts array of all of the positinos of the contours applied to the color image
        """
         i = 0
        count1 = 0
        for i in cnts:
            count2 = 0
            if (count1 % 80000000000 == 0):
                j = 0
                for j in i:
                    count3 = 0
                    if (count2% 800000000000 == 0):
                        k = 0
                        for k in j:
                            #print (k)
                            if(count3 % 800000000 == 0):
                                x = k[0]
                                y = k[1]
                                print(backbone.threePoint(depth_frame,x, y))
                        count3 = count3+1
                    count2 = count2+1
        cont1 = count1 + 1

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
        spatial.set_option(rs.option.holes_fill, 1)
        filtered_depth = spatial.process(frame)
        return filtered_depth

    def hole(self, frame):
        hole_filling = rs.hole_filling_filter()
        holeFilling = hole_filling.process(depth_frame)
        return holeFilling

    def threshold(self, frame, minDistance, maxDistance):
        #apply threshold filter to a depth image from the min distance to max distance to be viewed
        threshold_filter = rs.threshold_filter(minDistance, maxDistance)
        return threshold_filter.process(frame)

    def contour(self, depthFrame, color_image):
        """
        Apply the contour to the collor image. by determining the threshold applied to the depth image and apply that outer edges to the color image as a contour
        @param depthFrame frame that has threshold filter applied
        @param color_image color image feed thats in a numpy array
        @return cnts retruns a the postion of the contours in an array of [[[]]]
        """

        low = np.array([110, 110, 110])
        high = np.array([150, 150, 150])
        """
        In range deterine if the inputed array is within the range of the other two array
        and outputs an array
        """
        mask = cv2.inRange(depthFrame, low, high)
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(color_image, cnts,-1, [0,255,0], 2)
        return cnts






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
        #retrives the depth image from camera
        depth_frame = backbone.getDepthFrame(frames)
        # retrives color image a a np array
        color_image = backbone.colorImageCV2(frames)

        disparity = rs.disparity_transform(True)

        # apply the filteras and threshold filter to the image
        #depeneding on when you apply disparity it willl cause the threshold to not work apply after doing threshold
       
        depthFrame = backbone.hole(depth_frame)
        #depthFrame = disparity.process(depthFrame) 
        depthFrame = backbone.spatial(depthFrame)
        depthFrame = backbone.threshold(depthFrame, 2.5, 5)
        depthFrame = disparity.process(depthFrame) 
        #depthFrame = backbone.decimation(depthFrame)

        #depthImage = colorizer.colorize(depth_frame)
        #depthImage = np.asanyarray(depthImage.get_data())
        depthFrame = backbone.depthImageCV2(depthFrame)

        #apply the contour to the color image
        cnts = backbone.contour(depthFrame, color_image)
        
        
#        count = 0
#        for i in cnts:# goes trhough all the countours countour number 
#            for j in i:# goes through a for loop through all of i
#                if (count % 154564 == 0):
#                    x = j[0][0]
#                    y = j[0][1]
#                    print(backbone.threePoint(x, y))#prints the all of the piexl of each countor 
#                count = count + 1
                #make preset sequence of items to get the position from for each contour
                #returnuing an array.

        #contours are stored in an aray of array of array of array

        #number of contours can be from 2 to 1000
        #number of cordinates from 1-2000
        #total number of contours
        contour = len(cnts)
        #determine how much to increment throught the contour array based on the total number of contours
                                                                            #if (contour < 5):
                                                                            #   contourinc = 1
                                                                            #else:
                                                                            #    #if there more than 5. Determine the increment to vist 5 countorus evenly
                                                                            #    #round to nearest integer
                                                                            #    contourinc = int(round(contour/5))
        
                                                                            #contourindex = 0
                                                                            #size = len(cnts[0][0][0])
                                                                            #size2 = np.size(cnts[0][0][0][0])
                                                                            ##print (size, size2, cnts[0][0][0][0])                 
                                                                            ##print (cnts[0][0][0])
                                                                            ##print ("asd")

                                                                            ##while (contourindex < contour):
                                                                            ##    cordinate = len(cnts[contourindex])   #num cordinates
                                                                            ##    if (cordinate < 5):
                                                                            ##        cordinateinc = 1
                                                                            ##    else:
                                                                            ##        cordinateinc = int(round(cordinate/5))
                                                                            ##    cordinateindex = 0
                                                                            ##    while (cordinateindex < cordinate and contourindex < contour ):
                                                                            ##        if(cordinateindex < cordinate):
                                                                            ##            position = cnts[contourindex][contourindex][0]
                                                                            ##            x = position[0]
                                                                            ##            y = position[1]
                                                                            ##            cordinateindex = (cordinateindex + cordinateinc)
                                                                            ##            print(backbone.threePoint(x, y))

                                                                            ##    contourindex = (contourindex + contourinc)
        i = 0
        count1 = 0
        for i in cnts:
            count2 = 0
            if (count1 % 80000000000 == 0):
                j = 0
                for j in i:
                    count3 = 0
                    if (count2% 800000000000 == 0):
                        k = 0
                        for k in j:
                            #print (k)
                            if(count3 % 800000000 == 0):
                                x = k[0]
                                y = k[1]
                                print(backbone.threePoint(depth_frame,x, y))
                        count3 = count3+1
                    count2 = count2+1
        cont1 = count1 + 1

        #print ("aijojdwioajio")



        #get the size of the iamge
        #dimensions = image1.shape
        #print ("Image dimensions: ", dimensions)
        #position = backbone.threePoint(500, 500)
        #print(position)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        res = np.hstack((depthFrame, color_image))
        cv2.imshow('RealSense', res)
        cv2.waitKey(1)

        #end the program when the windows is closed
        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            break


pipeline.stop()


#work on countor going through the list thats made and processing increments of the pixels and return a 3d point look at getting th contour smother 
#list of pixel pointes