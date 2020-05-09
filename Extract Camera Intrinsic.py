#!/usr/bin/env python
# coding: utf-8

# In[8]:


import numpy as np
import pickle,os
from scipy.io import savemat


# In[9]:


filename = "cameras.txt"
outputCameraFeaturesDir = "Camera Parameters"
if not os.path.exists(outputCameraFeaturesDir):
    os.makedirs(outputCameraFeaturesDir)
    
outputIntrinsicFeatures = "cameraIntrinsic.txt"


# In[25]:


with open(filename, "r") as fp:
    allLines = fp.readlines()
    allLines = allLines[3:]
    for line in allLines:
        line  = line.split('\n')[0]
        items = line.split(' ')
        CAMERA_ID = int(items[0])
        MODEL     = items[1]
        WIDTH     = float(items[2])
        HEIGHT    = float(items[3])
        PARAMS    = items[4:]
        PARAMS    = [float(i) for i in PARAMS]
        focalLen  = PARAMS[0]
        x0        = PARAMS[1]
        y0        = PARAMS[2]
        gamma     = PARAMS[3]
        intrinsic = np.array([[focalLen,gamma,x0],[0,focalLen,y0],[0,0,1]])
        np.set_printoptions(precision=4,suppress=True)
        print (intrinsic)


# In[7]:


with open(os.path.join(outputCameraFeaturesDir,outputIntrinsicFeatures), 'wb') as fp:
    pickle.dump(intrinsic, fp)

