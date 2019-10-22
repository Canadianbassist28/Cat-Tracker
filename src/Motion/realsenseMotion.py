
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

        self.accelAngle = (0,0,0) ##pitch, yaw, roll(rads)
        self.position = (0,0,0) ##x,y,z (meters)
        self.velocity = (0,0,0) ##x,y,z (m/s)
        self.angle = (0,0,0) 

        ##linearAccel uses rotation angle to remove gravity component from accel
        self.linearAccel = (0,0,0) #x,y,z

        self.accelBias = (.56, 0.25, -1.3)
        self.accelSlope = (-1.3e-5, 1, 3.06e-5)

        self.accelBuf = []
        self.accelBufSize = 20

        self.frameCount = 0
        self.lastTime = 0

    def get_data(self,frames,time = None):
        """
        Extracts motion data from a frame and processes data into angle, veclocity, and position
        @param frames rs.composite_frames from pipeline.wait_for_frames()
        @param time current time in seconds, used if stream is playing back in non-realtime
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
        self.accel = (tmp.x + self.accelBias[0], tmp.y + self.accelBias[1], tmp.z + self.accelBias[2])

        #apply running average filter
        self.accel = self.__accelFilter(self.accel)

        #-------integrate gyro(rad/s) to get angle(rads) 
        tmp = self.__integrate(self.gyro, self.lastGyro, timeNow)
        self.angle = tuple(map(sum, zip(self.angle, tmp)))      

        if len(self.accelBuf) > 5:
            #-------calculate linearAccel by subtracting gravity component from accel
            R = self.__getRotationMatrix()
            g = np.array([[0], [9.81], [0]])
            gravityVector = np.transpose(np.matmul(R, g))[0]
            self.linearAccel = tuple(np.add(np.array(self.accel), gravityVector))

            #-------integrate linearAccel(m/s^2) to get velocity(m/s)
            tmp = self.__integrate(self.linearAccel, self.lastLinearAccel, timeNow)
            #tmp = np.matmul(np.array(tmp), R)
            self.velocity = tuple(map(sum, zip(self.velocity, tmp)))

            #-------integrate velocity(m/s) to get position(m) 
            tmp = self.__integrate(self.velocity, self.lastVelocity, timeNow)
            self.position = tuple(map(sum, zip(self.position, tmp)))

        #set last time !!!(must be done at end of this function)!!!
        self.lastTime = timeNow
        self.frameCount += 1

    def __integrate(self, data, lastData , timeNow):
        """ 
        Uses trapozoidal integration on a vector [x,y,z] 
        @param data newest data, a tuple (x,y,z)
        @param lastData last data, a tuple (x,y,z)
        @param timeNow current time in seconds (float)
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

    def __getAccelAngle(self):
        X = self.accel[1]
        Y = self.accel[2]
        Z = self.accel[0]   

        roll = np.arctan2(Y, Z)
        pitch = np.arctan2(-X, np.sqrt(Y**2 + Z**2))

        self.accelAngle = (roll, 0, pitch)
        #print(self.accelAngle, "|", self.angle)
        
    def __accelFilter(self, newData):
        if len(self.accelBuf) == self.accelBufSize:
            self.accelBuf.pop()
            self.accelBuf.append(newData)
        else:
            self.accelBuf.append(newData)
            #return newData

        result = (0,0,0)
        for i in self.accelBuf:
            result = tuple(map(sum, zip(result, i)))

        return [x/len(self.accelBuf) for x in result]