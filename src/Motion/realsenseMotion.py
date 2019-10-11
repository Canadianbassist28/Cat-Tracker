""" 
@package contains object realsenseMotion, used to handle motion data from the intelRealsense camera
@author Dylan Wright dw437013@ohio.edu
"""

import pyrealsense2 as rs
from time import clock as timer
from math import sqrt

class realsenseMotion(object):
    """
    Handles motion data from the realsense camera
    """
    def __init__(self):

        self.accel = (0,0,0)
        self.gyro = (0,0,0)

        self.lastGyro = (0,0,0)
        self.lastLinearAccel = (0,0,0)
        self.lastVelocity= (0,0,0)

        self.angle = (0,0,0) #roll, pitch, yaw (rads)
        self.position = (0,0,0) #x,y,z (meters)
        self.velocity = (0,0,0) #x,y,z (m/s)

        #linearAccel uses rotation angle to remove gravity component from accel
        self.linearAccel = (0,0,0) #x,y,z

        self.lastTime = timer()

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
    
        #set last data
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
        self.accel = (tmp.x, tmp.y, tmp.z)

        #integrate gyro(rad/s) to get angle(rads) 
        tmp = self.__integrate(self.gyro, self.lastGyro, timeNow)
        self.angle = tuple(map(sum, zip(self.angle, tmp)))

        #------------------------------------------------------------
        ###TODO: add noise reduction filter to the accelerometer data
        #------------------------------------------------------------

        #compute linearaccel by adding the (unit vector of the angle) * (the accel due to gravity)
        angleUnit = self.__getUnitVector(self.angle)
        angleUnit = [x * (9.81) for x in angleUnit]

        self.linearAccel = tuple(map(sum, zip( angleUnit, self.accel)))

        #integrate linearAccel(m/s^2) to get velocity(m/s)
        tmp = self.__integrate(self.linearAccel, self.lastLinearAccel, timeNow)
        self.velocity = tuple(map(sum, zip(self.velocity, tmp)))

        #integrate velocity(m/s) to get position(m) 
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
        width = (timeNow - self.lastTime)

        xrect = width * lastData[0]
        yrect = width * lastData[1]
        zrect = width * lastData[2]

        xtri = 0.5 * width * (data[0] - lastData[0])
        ytri = 0.5 * width * (data[1] - lastData[1])
        ztri = 0.5 * width * (data[2] - lastData[2])

        return ((xrect + xtri), (yrect + ytri), (zrect + ztri))

    def __getUnitVector(self, vector):
        """
        From a vector (x,y,z) returns the unit vector in the original vector direction
        @param vector a tuple (x,y,z)
        @return a tuple (ax, ay, az)
        """
        mag = sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)

        return [x/mag for x in vector] 