import pyrealsense2 as rs
import math as math
import numpy as np
from timeit import default_timer as timer
import plotly.plotly as py
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
    def init (self):
        self.xaxis= Motion.position[0]
        self.yaxis= Motion.position[1] 
        self.zaxis= Motion.position[2]
        self.xangle= Motion.angle[0]
        self.yangle= Motion.angle[1]
        self.zangle= Motion.angle[2]

        
        self.data=[()]

        
        #I will need a group of functions called Turn 
        #which is a boolean that return true if we turn left, right, up and down
        
    #-----this function will print the data type thats associated

    def load (self):
        #this function will take information from dylan's piece of the project and load it into a tuple to gain real time information.
        #this will be useful for mapping because the we can use it to map using this tuple
        while(rs.playback_status.playing):
            Motion.position=[(self.xaxis, self.yaxis, self.zaxis)]
            self.data[position, angle]


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
    x= Motion.xaxis
    y=Motion.yaxis
    xGrid, yGrid = np.meshgrid(y, x)
    R = np.sqrt(xGrid ** 2 + yGrid ** 2)
    z = np.sin(R)
    # Creating the plot
    lines = []
    line_marker = dict(color='#0066FF', width=2)
    for i, j, k in zip(xGrid, yGrid, z):
        lines.append(go.Scatter3d(x=i, y=j, z=k, mode='lines', line=line_marker))
    layout = go.Layout(
        title='Wireframe Plot',
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
    py.iplot(fig, filename='wireframe_plot')
        