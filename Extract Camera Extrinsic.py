#!/usr/bin/env python
# coding: utf-8

# In[83]:


import numpy as np
import pickle,os
from scipy.io import savemat
from scipy.spatial.transform import Rotation as R
import quaternion as qt


# In[84]:


inputImagefilename = "images.txt"
outputCameraFeaturesDir = "Camera Parameters"
if not os.path.exists(outputCameraFeaturesDir):
    os.makedirs(outputCameraFeaturesDir)


outputExtrinsicFeatures = "cameraExtrinsic.txt"
outputExtrinsicPoints   = "cameraExtrinsicPoints.txt"


# In[85]:


with open(inputImagefilename, "r") as fp:
    allLines = fp.readlines()
    allLines = allLines[4:]
    flag     = 0
    image    = {}
    imagePoints = {}
    for line in allLines:
        if flag==0:
            line  = line.split('\n')[0]
            items = line.split(' ')
            IMAGE_ID = int(items[0])
            QW    = float(items[1])
            QX    = float(items[2])
            QY    = float(items[3])
            QZ    = float(items[4])
            TX    = float(items[5])
            TY    = float(items[6])
            TZ    = float(items[7])
            CAMERA_ID = [int(items[8])]
            NAME  = items[9]
            
            rotationMat = qt.as_rotation_matrix(np.quaternion(QW,QX,QY,QZ))
            translationMat = np.asarray([TX,TY,TZ]).reshape(3,1)
            transformationMat = np.hstack((rotationMat,translationMat))
            image[NAME] = transformationMat
                        
            flag  = 1 
        else:
            allthings = line.split(' ')
            points = []
            index = 0
            while index < (len(allthings)-2):
                px = float(allthings[index])
                py = float(allthings[index+1])
                cons = int(allthings[index+2])
                points.append([px,py,cons])
                index = index+3
            imagePoints[NAME] = points
            flag = 0


# In[86]:


with open(os.path.join(outputCameraFeaturesDir,outputExtrinsicFeatures), 'wb') as fp:
    pickle.dump(image, fp)
with open(os.path.join(outputCameraFeaturesDir,outputExtrinsicPoints),'wb') as fp:
    pickle.dump(imagePoints,fp)

