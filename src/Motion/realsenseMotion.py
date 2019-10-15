
import pyrealsense2 as rs
import numpy as np
from time import clock as timer


class realsenseMotion(object):
    """
    Handles motion data from the realsense camera
    Must call get_data every frame loop to update motion data
    @author Dylan Wright dw437013@ohio.edu
    """    

    def __init__(self):

        self.accel = (0,0,0)
        self.gyro = (0,0,0)

        self.lastGyro = (0,0,0)
        self.lastLinearAccel = (0,0,0)
        self.lastVelocity= (0,0,0)

        self.Gyroangle = (0,0,0) ##pitch, yaw, roll(rads)
        self.position = (0,0,0) ##x,y,z (meters)
        self.velocity = (0,0,0) ##x,y,z (m/s)
        self.angle = (0,0,0) #result of complementaryFilter()

        ##linearAccel uses rotation angle to remove gravity component from accel
        self.linearAccel = (0,0,0) #x,y,z

        self.accelXBias = -.25
        self.accelYBias = .3125
        self.accelZBias = -.5

        self.lastTime = 0

    def get_data(self,frames,time = None):
        """
        Extracts motion data from a frame and processes data into angle, veclocity, and position
        @param frames the collection of frames from pipeline.wait_for_frames()
        @param time current time, used if stream is playing back in non-realtime
        """
        if time == None:
            timeNow = timer()
        else:
            timeNow = time
    
        #store last data
        self.lastGyro = self.gyro
        self.lastLinearAccel= self.linearAccel
        self.lastVelocity = self.velocity

        #retreve motion frames
        gyroFrame = frames.first_or_default(rs.stream.gyro).as_motion_frame()
        accelFrame = frames.first_or_default(rs.stream.accel).as_motion_frame()

        #retreive data from frame
        tmp = gyroFrame.get_motion_data()
        self.gyro = (tmp.x, tmp.y, tmp.z)
        tmp = accelFrame.get_motion_data()
        self.accel = (tmp.x , tmp.y , tmp.z )

        #-------integrate gyro(rad/s) to get angle(rads) 
        tmp = self.__integrate(self.gyro, self.lastGyro, timeNow)
        self.angle = tuple(map(sum, zip(self.angle, tmp)))
        
        #------------------------------------------------------------
        ###TODO: add complementary filter to the accelerometer data
        #------------------------------------------------------------

        #-------calculate linearAccel by subtracting gravity component from accel
        R = self.__getRotationMatrix()
        g = np.array([[0], [9.81], [0]])
        gravityVector = np.transpose(np.matmul(R, g))[0]
        self.linearAccel = tuple(np.add(np.array(self.accel), gravityVector))

        #-------integrate linearAccel(m/s^2) to get velocity(m/s)
        tmp = self.__integrate(self.linearAccel, self.lastLinearAccel, timeNow)
        self.velocity = tuple(map(sum, zip(self.velocity, tmp)))

        #-------integrate velocity(m/s) to get position(m) 
        tmp = self.__integrate(self.velocity, self.lastVelocity, timeNow)
        self.position = tuple(map(sum, zip(self.position, tmp)))

        #set last time !!!(must be done at end of this function)!!!
        self.lastTime = timeNow

    def __integrate(self, data, lastData , timeNow):
        """ 
        Uses trapozoidal integration on a vector [x,y,z] 
        @param data newest data, a tuple (x,y,z)
        @param lastData last data, a tuple (x,y,z)
        @param timeNow current time (float)
        @return a tuple (x,y,z)
        """
        if self.lastTime != 0:
            width = (timeNow - self.lastTime)
        else: 
            return 0,0,0

        xrect = width * lastData[0]
        yrect = width * lastData[1]
        zrect = width * lastData[2]

        xtri = 0.5 * width * (data[0] - lastData[0])
        ytri = 0.5 * width * (data[1] - lastData[1])
        ztri = 0.5 * width * (data[2] - lastData[2])

        return ((xrect + xtri), (yrect + ytri), (zrect + ztri))


    def __getRotationMatrix(self):
        """
        From the current angle in eulars angles, computes the rotation matrix 
        @return the rotation matrix, a np.array matrix
        """
        theta = self.angle

        R_x = np.array([[1, 0,                                0],
                        [0, np.cos(theta[0]), -np.sin(theta[0])],
                        [0, np.sin(theta[0]),  np.cos(theta[0])]])

        R_y = np.array([[np.cos(theta[1]),    0, np.sin(theta[1])],
                        [0,                   1, 0               ],
                        [-np.sin(theta[1]),   0, np.cos(theta[1])]])

        R_z = np.array([[np.cos(theta[2]),   -np.sin(theta[2]), 0],
                        [np.sin(theta[2]),    np.cos(theta[2]), 0],
                        [0,                   0,                1]])

        R = np.dot(R_z, np.dot( R_y, R_x ))

        return R

    def __getUnitVector(self, vector):
        """
        From a vector (x,y,z) returns the unit vector in the original vector direction
        @param vector a tuple (x,y,z)
        @return a tuple (ax, ay, az)
        """
        mag = np.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2) * -1

        return [x/mag for x in vector] 