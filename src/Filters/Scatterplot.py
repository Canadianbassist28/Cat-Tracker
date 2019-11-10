#make threeD plot of all the poitns with matplot lib.
#scatter plot
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


listx = []
listy = []
listz = []

#open the file and read in the data to a list
file = open("output.txt", "r")
list1 = []
i = file.readline()

for i in file:
#finds x, y,z points converts to float and puts it into coresponding list
    index_x = i.find(',' , 0)
    listx.append(float(i[1:index_x]))
    index_y = i.find(',' , index_x +1, len(i))
    listy.append(float(i[index_x + 1: index_y]))
    listz.append(float(i[index_y+2 : len(i)-2]))

xasarray = np.asarray(listx)
yasarray = np.asarray(listy)
zasarray = np.asarray(listz)

#print (list1[0])

#conver to an array

fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d') #add 3rd axes
#plt.scatter(xasarray,yasarray)
ax.scatter(xasarray, yasarray, zasarray) #fix parameters sending.
plt.show()