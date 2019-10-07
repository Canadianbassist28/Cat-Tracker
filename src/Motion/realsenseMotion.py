import pyrealsense2 as rs

class realsenseMotion(object):
	"""object to handle motion data from the realsense"""
	def __init__(self):
		self.accelData = None
		self.gyroData = None

	def get_motion_frames(self,frames):
		for frame in frames:
			profile = frame.get_profile()
			if profile.stream_type() == rs.stream.accel and profile.format() == rs.format.motion_xyz32f:
					self.accelData = frame.as_motion_frame().get_motion_data()

			if profile.stream_type() == rs.stream.gyro and profile.format() == rs.format.motion_xyz32f:   
					self.gyroData = frame.as_motion_frame().get_motion_data()
	
	def __repr__(self):
		return ("Accelerometer: {}\n".format(self.accelData)) + ("Gyroscope: {}".format(self.gyroData))