import pyrealsense2 as rs
from time import clock as timer

class realsenseMotion(object):
    """Handles motion data from the realsense camera"""
    def __init__(self):
        _timeNow = timer()

        self.accelData = rs.vector()
        self.gyroData = rs.vector()
        self.VelData = (0,0,0)

        self.lastaccelData = rs.vector()
        self.lastGyroData = rs.vector()
        self.lastVelData = rs.vector()

        self.lastVelTime = _timeNow
        self.lastaccelTime = _timeNow
        self.lastGyroTime = _timeNow

        self.angle = (0,0,0) #roll, pitch, yaw
        self.position = (0,0,0) #x,y,z

    def get_data(self,frames):
        """
        Extracts motion data from a frame and processes data into angle and position
        @param frames the collection of frames from pipeline.wait_for_frames()
        """
        timeNow = timer()

        #set last data
        self.lastGyroData = self.gyroData
        self.lastaccelData = self.accelData

        #retreve motion frames
        gyroFrame = frames.first_or_default(rs.stream.gyro).as_motion_frame()
        accelFrame = frames.first_or_default(rs.stream.accel).as_motion_frame()

        #retreive data from frame
        self.gyroData = gyroFrame.get_motion_data()
        self.lastGyroTime = timeNow
        self.accelData = accelFrame.get_motion_data()
        self.lastaccelTime = timeNow

        #integrate gyroData(rad/s) to get angle(rads) 
        tmp = self.__integrate(self.gyroData, self.lastGyroData, self.lastGyroTime)
        self.angle = tuple(map(sum, zip(self.angle, tmp)))

    def __integrate(self, data, lastData, lastTime):
        """ 
        Uses trapozoidal integration on a vector [x,y,z] 
        @param data newest data, (rs.vector) 
        @param lastData last data (rs.vector)
        @param lastTime last time (float)
        @return a tuple (x,y,z)
        """
        width = (timer() - lastTime)

        xrect = width * lastData.x
        yrect = width * lastData.y
        zrect = width * lastData.z

        xtri = 0.5 * width * (data.x - lastData.x)
        ytri = 0.5 * width * (data.y - lastData.y)
        ztri = 0.5 * width * (data.z - lastData.z)

        return ((xrect + xtri), (yrect + ytri), (zrect + ztri))