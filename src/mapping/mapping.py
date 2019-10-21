import pyrealsense2 as rs
import numpy as np

from src.motion.realsenseMotion import realsenseMotion as dylanMotion


class realsenseMap(object):

    """ this class realsense Map is going to take the data from dylan's
    motion class. Dylans class takes a specific point in time 
    and records objects at that specific distance.
    
    My class get multiple instances and turn it into a larger object.
    It will also be able to read a specific point and calculate distance and rotation 
    """
    def init (self):
