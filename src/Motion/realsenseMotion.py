import pyrealsense2 as rs
from timeit import default_timer as timer

class realsenseMotion(object):
	"""object to handle motion data from the realsense"""
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

		timeNow = timer()

		self.lastGyroData = self.gyroData
		self.lastaccelData = self.accelData

		for frame in frames:
			profile = frame.get_profile()
			if profile.stream_type() == rs.stream.accel and profile.format() == rs.format.motion_xyz32f:
					self.accelData = frame.as_motion_frame().get_motion_data()
					self.lastaccelTime = timeNow


			if profile.stream_type() == rs.stream.gyro and profile.format() == rs.format.motion_xyz32f:   
					self.gyroData = frame.as_motion_frame().get_motion_data()
					self.lastGyroTime = timeNow

		tmp = self.__integrate(self.gyroData, self.lastGyroData, self.lastGyroTime)
		self.angle = tuple(map(sum, zip(self.angle, tmp)))

	
	def __integrate(self, data, lastData, lastTime):
		""" Uses trapozoidal integration on a vector [x,y,z] 
			@return a tuple (x,y,z)"""

		width = timer() - lastTime

		xrect = width * lastData.x
		yrect = width * lastData.y
		zrect = width * lastData.z

		xtri = 0.5 * width * (data.x - lastData.x)
		ytri = 0.5 * width * (data.y - lastData.y)
		ztri = 0.5 * width * (data.z - lastData.z)

		return ((xrect + xtri), (yrect + ytri), (zrect + ztri))

	