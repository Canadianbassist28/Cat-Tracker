import pyrealsense2 as rs
import math as math
import numpy as np
from timeit import default_timer as timer
import chart_studio.plotly as py
import plotly.graph_objs as go
from src.Motion.realsenseMotion import realsenseMotion as Motion


class realsenseMap(object):

    """ 
    this class realsense Map is going to take the data from dylan's
    motion class. Dylans class takes a specific point in time 
    and records objects at that specific distance.
    
    My class get multiple instances and turn it into a larger object.
    It will also be able to read a specific point and calculate distance
   and rotation 
    """
    def __init__ (self):
        self.xaxis=0
        self.yaxis=0
        self.zaxis=0
        self.angle=0 
        self.xcontour=0
        self.ycontour=0
        self.zcontour=0

        
        self.data = []

        """ 
        I will need a group of functions called Turn 
        which is a boolean that return true if we turn left, right, up and down
        """
    """-----this function will print the data type thats associated  """

    def load (self, position, angle, threeDpoints):
        #this function will take information from dylan's piece of the project and load it into a tuple to gain real time information.
        #this will be useful for mapping because the we can use it to map using this tuple
      
        for i in threeDpoints:
            self.xcontour = threeDpoints(i[0])
            self.ycontour = threeDpoints(i[1])
            self.zcontour = threeDpoints(i[2])

        self.xaxis = position[0]
        self.yaxis = position[1]
        self.zaxis = position[2]
        tup = (position[0], position[1], position[2])
        self.data.append(tup)




    def print(self):
   #this function will be able to print the map to a file
        print(self.data)

    def distance(self):
   #this function will be able to calculate the distance from the initial point and create a tuple of distance from any point while the program is running 
        tot_distance= (math.sqrt((self.xaxis-init.self.xaxis)^2), math.sqrt((self.yaxis-init.self.yaxis)^2), math.sqrt((self.zaxis-init.self.zaxis)^2))
        return tot_distance

    def isTurn(self):
    #this function will be able to decide if the camera has turned using the gyroscope
        if turnLeft(self) == true or turnRight(self) == true:  
             return True
        else: 
             return False
    
    def turnLeft(self):
    #this function will identify a left turn
         if self.xaxis > self.data.xaxis and self.angle > 270 and self.angle < 360:
            return True
         else: 
            return False
    def turnRight(self):
    #this function will display a right turn
        if self.xaxis < self.data.xaxis and self.angle > 45 and self.angle < 90:
            return True
        else: 
            return False
# Creating the data
    def wireframe(self):
        x = []
        y = []
        for i in self.data:
            x.append(i[0])
            y.append(i[1])
        xGrid, yGrid = np.meshgrid(y, x)
        R = np.sqrt(xGrid ** 2 + yGrid ** 2)
        z = np.sin(R)

        # Creating the plot
        lines = []
        line_marker = dict(color='#0066FF', width=2)
        for i, j, k in zip(xGrid, yGrid, z):
            lines.append(go.Scatter3d(x=i, y=j, z=k, mode='lines', line=line_marker))
        layout = go.Layout(
            title='Cat-Tracker Map',
            scene=dict(
                xaxis=dict(
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                yaxis=dict(
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                ),
                zaxis=dict(
                    gridcolor='rgb(255, 255, 255)',
                    zerolinecolor='rgb(255, 255, 255)',
                    showbackground=True,
                    backgroundcolor='rgb(230, 230,230)'
                )
            ),
            showlegend=False,
        )
        fig = go.Figure(data=lines, layout=layout)
        fig.show()
        py.iplot(fig, filename='wireframe_plot')

        