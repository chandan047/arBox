#!/usr/bin/env python
# coding: utf-8

# In[19]:


import numpy as np
import pickle
from scipy.io import savemat
import os


# In[20]:


filename = "points3D.txt"
output_filename = "3DPointCloud.txt"
output_filename_mat = "3DPointCloud.mat"
outputDir = "COLMAP 3D Points"
if not os.path.exists(outputDir):
        os.makedirs(outputDir)


# In[21]:


'''
READ THE COLMAP GENERATED POINTS FROM "points3D.txt" AND EXTRACT THE [X,Y,Z] COORDINATES FOR ALL POINTS
'''

with open(filename, "r") as fp:
    allLines = fp.readlines()
    allLines = allLines[3:]
    allPoints = []
    for line in allLines:
        indElements  = line.split(' ')
        POINT3D_ID = int(indElements[0])
        X,Y,Z = float(indElements[1]),float(indElements[2]),float(indElements[3])
        allPoints.append([POINT3D_ID,X,Y,Z])


# In[22]:


with open(os.path.join(outputDir,output_filename), 'wb') as fp:
    pickle.dump(allPoints, fp)


# In[23]:


allPoints = np.asarray(allPoints)
savemat(os.path.join(outputDir,output_filename_mat), {"points":allPoints})

