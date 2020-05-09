#!/usr/bin/env python
# coding: utf-8

# In[26]:


import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle
from scipy.io import savemat
import random,os


# In[29]:


input3DpointsDir        = "COLMAP 3D Points"
ransacOutputDir         = "RANSAC Outputs"
if not os.path.exists(input3DpointsDir):
    os.makedirs(input3DpointsDir)
if not os.path.exists(ransacOutputDir):
    os.makedirs(ransacOutputDir)

ransacFittedInliers     = "planeFitPoints.txt"
ransacFittedInliers_mat = "planeFitPoints.mat"
ransacFittedABCD        = "planeFitCoeff.txt"
ransacFittedABCD_mat    = "planeFitCoeff.mat"

input3DpointCloudFilename = "3DPointCloud.txt"
with open (os.path.join(input3DpointsDir,input3DpointCloudFilename), 'rb') as fp:
    data = pickle.load(fp)


# In[28]:


def ransac(data, estimate, is_inlier, sample_size, goal_inliers, max_iterations, stop_at_goal=True, random_seed=None):
    soFarBestIC = 0
    soFarBestPlane = None
    random.seed(random_seed)
    soFarBestSetPoints = []
    data = list(data)
    for i in range(max_iterations):
        randomSample = random.sample(data, int(sample_size))
        currPlane = estimate(randomSample)
        currIC = 0
        
        currSetPoints = []
        for j in range(len(data)):
            if is_inlier(currPlane, data[j]):
                currIC += 1
                currSetPoints.append(data[j])
                
        print('estimate:', currPlane)
        print('# inliers:', currIC)

        if currIC > soFarBestIC:
            soFarBestIC = currIC
            soFarBestPlane = currPlane
            soFarBestSetPoints = currSetPoints
            if currIC > goal_inliers and stop_at_goal:
                break
    print('took iterations:', i+1, 'best model:', soFarBestPlane, 'explains:', soFarBestIC)
    return soFarBestPlane, soFarBestIC, soFarBestSetPoints


# In[4]:


def augment(xyzs):
    axyz = np.ones((len(xyzs), 4))
    axyz[:, :3] = xyzs
    return axyz

def estimate(xyzs):
    axyz = augment(xyzs[:3])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold

def plot_plane(a, b, c, d):
    xx, yy = np.mgrid[:10, :10]
    return xx, yy, (-d - a * xx - b * yy) / c


# In[5]:


numPoints = len(data)
print ("Total points we have in the 3D point cloud are - {}".format(numPoints))
max_iter  = 5000
req_inliers = numPoints * 0.8
random_seed = 2020

data = np.asarray(data)
pointIds = data[:,0] 
xyzs = data[:,1:]

# RANSAC
m, b, s = ransac(xyzs, estimate, lambda x, y: is_inlier(x, y, 0.09), 3, req_inliers, max_iter,random_seed)


# In[6]:


m,b,s   = ransac(s, estimate, lambda x, y: is_inlier(x, y, 0.02), 3, req_inliers, max_iter,random_seed)


# In[11]:


a, b, c, d = m


# In[25]:


allPoints = np.asarray(s)
savemat(os.path.join(ransacOutputDir,ransacFittedInliers_mat),{"points":allPoints})
with open(os.path.join(ransacOutputDir,ransacFittedInliers),'wb') as fp:
    pickle.dump(allPoints,fp)


# In[30]:


plane = [a,b,c,d]
with open(os.path.join(ransacOutputDir,ransacFittedABCD), 'wb') as fp:
    pickle.dump(plane, fp)
plane = np.asarray(plane)
savemat(os.path.join(ransacOutputDir,ransacFittedABCD_mat), {"coeffs":plane})


# In[ ]:




