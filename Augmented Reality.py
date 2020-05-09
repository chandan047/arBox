#!/usr/bin/env python
# coding: utf-8

# In[50]:


import numpy as np
import pickle
from scipy.io import savemat
from matplotlib.image import imread
from PIL import Image
from matplotlib import pyplot as plt
import matplotlib
import os
import scipy.io as sio
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


# In[51]:


ransacInliersDir         = "RANSAC Outputs"
arOutputImageDir         = "Final images"
intputCameraFeaturesDir = "Camera Parameters"
imageDir                 = "images"
scene3DboxInputDir       = "3D Box"

if not os.path.exists(arOutputImageDir):
    os.makedirs(arOutputImageDir)

intrinsicFilename = "cameraIntrinsic.txt"
extrinsicFilename = "cameraExtrinsic.txt"
extrinsicFilenameForPoints = "cameraExtrinsicPoints.txt"


inlierPointsFile = "planeFitPoints.txt"
boxPointsFileName = "sceneBox3D.mat"


# In[52]:


def convert2D(point3D, K,Rt):
    trans2Dpoint =np.matmul(np.matmul(K,Rt),point3D)
    factor = trans2Dpoint[2][0]
    point2D = np.asarray([trans2Dpoint[0][0]/factor,trans2Dpoint[1][0]/factor])
    return point2D


# In[53]:


with open(os.path.join(intputCameraFeaturesDir,intrinsicFilename),'rb') as fp:
    intrinsic = pickle.load(fp)
with open(os.path.join(intputCameraFeaturesDir,extrinsicFilename),'rb') as fp:
    extrinsicAll = pickle.load(fp)
with open(os.path.join(intputCameraFeaturesDir,extrinsicFilenameForPoints),'rb') as fp:
    extrinsicImagePoints = pickle.load(fp)
with open(os.path.join(ransacInliersDir,inlierPointsFile),'rb') as fp:
    inlierPoints = pickle.load(fp)
boxPoints3D = sio.loadmat(os.path.join(scene3DboxInputDir,boxPointsFileName))


# In[54]:


# EXTRACT ALL IMAGES AS A DICT OF MATRICES

allImages = {}
for (dirpath, dirnames, filenames) in os.walk(imageDir):
    for file in filenames:
        imagePath = os.path.join(dirpath,file)
        image = imread(imagePath)
        allImages[file] = image


# In[30]:


# BASE IMAGE PICTURES

for name,image in allImages.items():
    plt.figure(figsize=(15,15))
    plt.imshow(allImages[name], interpolation='nearest')


# In[62]:


# 3D POINT CLOUD EXTRACTED BY COLMAP
c=0
for name,image in allImages.items():
    c=c+1
    plt.figure(figsize=(15,15))
    plt.imshow(allImages[name], interpolation='nearest')
    
    plottedPoints = []
    currImagePoints = extrinsicImagePoints[name]
    for point in currImagePoints:
        if point[2]!=-1:
            plottedPoints.append(point)
    plottedPoints = np.asarray(plottedPoints)
    x = plottedPoints[:,0]
    y = plottedPoints[:,1]
    plt.scatter(x,y,s=6.0,c='r')
    plt.show()
    if (c==15):
        break


# In[65]:


# BASE INLIERS EXTRACTED WHEN PROJECTED ON THE IMAGE TO FIND THE PRESSURE POINTS

inlierPointX = inlierPoints[:,0]
inlierPointY = inlierPoints[:,1]
inlierPointZ = inlierPoints[:,2]

c=0
for name,image in allImages.items():
    c = c+1
    inlierPoint2Dx = []
    inlierPoint2Dy = []
    plt.figure(figsize=(15,15))
    plt.imshow(allImages[name], interpolation='nearest')
    currImageDims    = np.shape(image)
    currentExtrinsic = extrinsicAll[name]
    
    allBoxPoints = boxPoints3D['wireframe']
    allBoxPointsNew = []
    
    for index in range(len(inlierPointX)):
        point = np.asarray([inlierPointX[index],inlierPointY[index],inlierPointZ[index],1])
        point3D = np.reshape(point,(4,1))
        point2D = convert2D(point3D,intrinsic,currentExtrinsic)
        if ((point2D[0]>= 0 and point2D[0] <= currImageDims[1]) and (point2D[1]>= 0 and point2D[1] <= currImageDims[0])):
            inlierPoint2Dx.append(point2D[0])
            inlierPoint2Dy.append(point2D[1])
    for i in range(len(allBoxPoints)):
        boxPt = allBoxPoints[i,:]
        boxPt = np.hstack((boxPt, [1]))
        boxPt = np.reshape(boxPt,(4,1))
        boxPoints2D = convert2D(boxPt,intrinsic,currentExtrinsic)
        allBoxPointsNew.append(boxPoints2D)
        
    allBoxPointsNew = np.asarray(allBoxPointsNew)
#     plt.scatter(inlierPoint2Dx,inlierPoint2Dy,s=2,c='g')
    plt.scatter(allBoxPointsNew[:,0],allBoxPointsNew[:,1],s=2.2,c='w')
    plt.show()
    if c==15:
        break


# In[59]:


# PLOTTING THE BOX OVER THE ORIGINAL IMAGE

inlierPointX = inlierPoints[:,0]
inlierPointY = inlierPoints[:,1]
inlierPointZ = inlierPoints[:,2]

c=0
for name,image in allImages.items():
    c = c+1
    inlierPoint2Dx = []
    inlierPoint2Dy = []
    fig = plt.figure(figsize=(15,15))
    ax  = plt.axes() 
    plt.imshow(allImages[name], interpolation='nearest')
    currImageDims    = np.shape(image)
    currentExtrinsic = extrinsicAll[name]
    
    lowerBoxPoints = boxPoints3D['lowerPoints']
    lowerBoxPointsNew = []
    upperBoxPoints = boxPoints3D['upperPoints']
    upperBoxPointsNew = []
    
    patches = []
    num_polygons = 2
    
    for index in range(len(inlierPointX)):
        point = np.asarray([inlierPointX[index],inlierPointY[index],inlierPointZ[index],1])
        point3D = np.reshape(point,(4,1))
        point2D = convert2D(point3D,intrinsic,currentExtrinsic)
        if ((point2D[0]>= 0 and point2D[0] <= currImageDims[1]) and (point2D[1]>= 0 and point2D[1] <= currImageDims[0])):
            inlierPoint2Dx.append(point2D[0])
            inlierPoint2Dy.append(point2D[1])
    for i in range(len(lowerBoxPoints)):
        boxPt = lowerBoxPoints[i,:]
        boxPt = np.hstack((boxPt, [1]))
        boxPt = np.reshape(boxPt,(4,1))
        boxPoints2D = convert2D(boxPt,intrinsic,currentExtrinsic)
        lowerBoxPointsNew.append(boxPoints2D)
    
    for i in range(len(upperBoxPoints)):
        boxPt = upperBoxPoints[i,:]
        boxPt = np.hstack((boxPt, [1]))
        boxPt = np.reshape(boxPt,(4,1))
        boxPoints2D = convert2D(boxPt,intrinsic,currentExtrinsic)
        upperBoxPointsNew.append(boxPoints2D)

    
    polygon = Polygon([lowerBoxPointsNew[0],lowerBoxPointsNew[1],lowerBoxPointsNew[2],lowerBoxPointsNew[3]], True, color = 'b')
    patches.append(polygon)
    
    polygon = Polygon([lowerBoxPointsNew[0],lowerBoxPointsNew[1],upperBoxPointsNew[1],upperBoxPointsNew[0]], True, color = 'g')
    patches.append(polygon)
    
    polygon = Polygon([lowerBoxPointsNew[1],lowerBoxPointsNew[2],upperBoxPointsNew[2],upperBoxPointsNew[1]], True, color = 'r')
    patches.append(polygon)
    
    polygon = Polygon([lowerBoxPointsNew[2],lowerBoxPointsNew[3],upperBoxPointsNew[3],upperBoxPointsNew[2]], True, color = 'w')
    patches.append(polygon)
    
    polygon = Polygon([lowerBoxPointsNew[0],lowerBoxPointsNew[3],upperBoxPointsNew[3],upperBoxPointsNew[0]], True, color = 'y')
    patches.append(polygon)
    
    polygon = Polygon([upperBoxPointsNew[0],upperBoxPointsNew[1],upperBoxPointsNew[2],upperBoxPointsNew[3]], True, color = 'm')
    patches.append(polygon)    
    
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=1)
#     colors = 100*np.random.rand(len(patches))
#     p.set_array(np.array(colors))
    p.set_color(['b','g','r','w','y','m'])
    ax.add_collection(p)
    
    arImgaeFileName = os.path.join(arOutputImageDir,name)
    plt.savefig(arImgaeFileName)
#     plt.show()
    plt.close()


# In[ ]:




