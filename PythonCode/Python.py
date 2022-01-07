#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#Matthew Badal-Badalian
#400187878
#badalbam
#Python Version 3.6.6

#Imports libaries
import numpy as np
import open3d as o3d

#Reads the x,y,z values from the xyz file
pointCloud = o3d.io.read_point_cloud("2dx4hallwaydatapoints.xyz", format = 'xyz')

#Variables initialization
currentPoint = 0
nextPoint = 0
plane = 0
lines = []

#Create planes
for currentPlane in range(20):
    for currentPoint in range(512):
        nextPoint = currentPoint + 1
        if(currentPoint == 511):
            nextPoint = 0
        lines.append([currentPoint+plane,(nextPoint)+plane])
    plane += 512;

#Reset variables
currentPoint = 0
plane = 0
do = 512

#Connect lines
for currentPlane in range(19):
    for currentPoint in range(512):
        lines.append([currentPoint+plane,currentPoint+do+plane])
    plane += 512;

#Creates the mesh with the xyz values    
line_set = o3d.geometry.LineSet(points = o3d.utility.Vector3dVector(np.asarray(pointCloud.points)), lines = o3d.utility.Vector2iVector(lines))

#Show results
o3d.visualization.draw_geometries([line_set])


# In[ ]:





# In[ ]:





# In[ ]:




