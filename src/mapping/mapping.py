import pyrealsense2 as rs
import math as math
import numpy as np
from timeit import default_timer as timer

from src.motion.realsenseMotion import realsenseMotion as Motion


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

        
        self.data=[()]

        """ 
        I will need a group of functions called Turn 
        which is a boolean that return true if we turn left, right, up and down
        """
    """-----this function will print the data type thats associated  """

    def load (self):
        #this function will take information from dylan's piece of the project and load it into a tuple to gain real time information.
        #this will be useful for mapping because the we can use it to map using this tuple
        while(is rs.playing)
            Motion.self.position=(self.xaxis, self.yaxis, self.zaxis)
            self.data[position, angle]

   def print(self.data)
   #this function will be able to print the map to a file
    print self.xaxis
    print self.yaxis
    print self.zaxis
    print self.axis

   def distance(self.data)
   #this function will be able to calculate the distance from the initial point and create a tuple of distance from any point while the program is running 
    distance= (math.sqrt((self.xaxis-init.self.xaxis)^2), math.sqrt((self.yaxis-init.self.yaxis)^2), math.sqrt((self.zaxis-init.self.zaxis)^2))
    
    def isTurn(self.data)
    #this function will be able to decide if the camera has turned using the gyroscope
    
        