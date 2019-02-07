'''
Created on Nov 2, 2015

@author: ash
'''

# import libraries
import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.interpolate import splprep, splev
from numpy import linspace
from optparse import OptionParser
import sys
import os
import pysal

_alphabetSize = 72
binSize = int(360 / _alphabetSize)

trajectoryRawDataFileURL = '/home/ash/Data/Kitti/BoilerPlate/txyzo.dat'

def genFeat(slpAngle, dictSize):
    slpAngle = slpAngle % 360
#     if abs(slpAngle) > 360:
#         slpAngle = slpAngle % 360
#         pass
    
    if slpAngle < 0:
        slpAngle += 360
        pass
    
    slpAngle = abs(slpAngle)
    if math.isnan(slpAngle):
        slpAngle = 0
    feat = int(slpAngle // binSize)
    return feat
    pass

if __name__ == '__main__':
    rawData = np.loadtxt(trajectoryRawDataFileURL, dtype=np.float, delimiter=',', skiprows=2, usecols=(1,2))
    print rawData
    horz = rawData[:,0]
    vert = rawData[:,1]
    plt.scatter(rawData[:,0], rawData[:,1], s=4, c='r', marker='.' )
    
    horz = horz[::10]
    vert = vert[::10]
    
    plt.scatter(rawData[:,0], rawData[:,1], s=1, c='k', marker='x' )
    
    _distList = []
    for i in xrange(len(horz)-1):
        _dist = math.sqrt((horz[i]-horz[i+1])**2 + (vert[i]-vert[i+1])**2)
        _distList.append(_dist)
        pass
    
    print np.average(_distList)
    print np.std(_distList)
    
    # spline parameters
    s=0.0 # smoothness parameter
    k=4 # spline order
    
    x = horz
    y = vert
    
#     M = 2*( len(x) + k)
    M = len(x)
    
    
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    z = t
    
    
    # find the knot points
    tckp,u = splprep([x,y,z],s=s,k=k,nest=-1)
    
    # evaluate spline, including interpolated points
    xnew,ynew,znew = splev(linspace(0,1,M),tckp)
    dx,dy,dz = splev(linspace(0,1,M), tckp, der=1)
    
    slp = []
    cnv = 180/ np.pi
    for i in xrange(len(dx)):
        slp.append(np.arctan((dy[i]/dx[i]))*cnv )
        pass
    
    # quantize the slope and assign to edge descriptor list
    roadLetFeat = []
    for elem in slp:
        feat = genFeat(elem, _alphabetSize)
        roadLetFeat.append(feat)
        pass
    
    print roadLetFeat
    print len(x), len(roadLetFeat)
    
    plt.show()
    pass