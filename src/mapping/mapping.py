import pyrealsense2 as rs
import numpy as np

from src.motion.realsenseMotion import realsenseMotion as dylanMotion


class realsenseMap(object):

    """ 
    this class realsense Map is going to take the data from dylan's
    motion class. Dylans class takes a specific point in time 
    and records objects at that specific distance.
    
    My class get multiple instances and turn it into a larger object.
    It will also be able to read a specific point and calculate distance
   and rotation 
    """
    def init (self):
        self.xaxis=0
        self.yaxis=0
        self.zaxis=0
        self.angle=0 

        self.data = [0,0,0,0]

        """ 
        I will need a group of functions called Turn 
        which is a boolean that return true if we turn left, right, up and down
        """
        
        """
        I will also need function called distance which 
        will calulate distance from a wall
        """
        