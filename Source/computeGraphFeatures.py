'''
Created on Nov 20, 2015
Compute the features of the graph acquired from OSM and write to shp/gpickle format in output
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
import geopy
from geopy.distance import vincenty

parser = OptionParser()
parser.add_option('-i', '--input', type='string', metavar='input', dest='input', default='karlsruheOSM', help='input osm file data directory name')
parser.add_option('-t','--fileType', type='string', metavar='fileType', dest='fileType', default='gpickle', help='input osm data file extension type')

inputFileDir = '/home/ash/Data/InputShapeFiles/'
outputFileDir = '/home/ash/Data/FixedShapeFiles/'

_alphabetSize = 72
binSize = int(360 / _alphabetSize)



def getEdgePoints(roadEdge):
    roadEdgeData = roadEdge[2]
    edgeKeyList = roadEdgeData.keys()
    edgePointStr = roadEdgeData['Wkt']
    b1 = edgePointStr.find('(')
    b2 = edgePointStr.find(')')
    edgePointsSubStr = edgePointStr[b1+1:b2]
    
    edgePointPairStrLst = edgePointsSubStr.split(',')
    edgePointPairLst = []
    for edgePointPairStr in edgePointPairStrLst:
        edgePointPair = [float(x) for x in edgePointPairStr.split(' ')]
        edgePointPairLst.append(edgePointPair)
        pass
    
    return edgePointPairLst
    pass

def getUniformSampledPoints(_r,_c):
    x = _r
    y = _c
    
    M = 5000
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    # ----------------------------------------------------------
    tol = 0.00001
    # ----------------------------------------------------------
    approxEdgeLength = math.sqrt((x[0]-x[-1])**2 + (y[0]-y[-1])**2)
    
    if approxEdgeLength/5 < tol:
        tol = approxEdgeLength/5
        pass

    i, idx = 0, [0]
    while i < len(x):
        total_dist = 0
        for j in range(i+1, len(x)):
            total_dist += math.sqrt((x[j]-x[j-1])**2 + (y[j]-y[j-1])**2)
            if total_dist > tol:
                idx.append(j)
                break
        i = j+1
    idx.append(len(x)-1)
    xn = x[idx]
    yn = y[idx]
    return xn,yn


def getMeasuredSamplePoints(_r,_c):
    x = _r
    y = _c
    
    _dist = 0
    for i in xrange(len(x)-1):
        node1 = (x[i],y[i])
        node2 = (x[i+1],y[i+1])
#         _dist += geopy.distance.vincenty(node1,node2).meters              # changes from vincenty to great_circle
        _dist += geopy.distance.great_circle(node1,node2).meters
        pass
    
    M = int(_dist)
    
    if M < len(x):
        M = len(x)
    
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    return x,y


def genFeat(slpAngle, dictSize):
    slpAngle = slpAngle % 360

    if slpAngle < 0:
        slpAngle += 360
        pass
    
    slpAngle = abs(slpAngle)
    if math.isnan(slpAngle):
        slpAngle = 0
    feat = int(slpAngle // binSize)
    return feat
    pass

def updateContourEdgeInfo(inputFileURL, outputFileURL):
    roadGraph = nx.read_gpickle(inputFileURL)
    
    _edgeList = nx.to_edgelist(roadGraph)
    contourAttribute = dict()
    
    # spline parameters
    s=0.0 # smoothness parameter
    k=4 # spline order
    
    for _edge in _edgeList:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
#         _r2,_c2 = getUniformSampledPoints(_r,_c)
        _r2,_c2 = getMeasuredSamplePoints(_r,_c)
        
        x = _r2
        y = _c2
#         M = 2*( len(x) + k)
        M = len(x)
        
        if M <= k:
            M = 2*k
        
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
        
        contourAttribute[(_edge[0],_edge[1])] = roadLetFeat
        pass
    
    nx.set_edge_attributes(roadGraph, 'contour', contourAttribute)
    nx.write_gpickle(roadGraph, outputFileURL)
    pass

def computeFeatures():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    
    inputDirURL = inputFileDir + options.input + '/'
    outputDirURL = outputFileDir + options.input + '/'
    
    if not os.path.exists(inputDirURL):
        os.mkdir(inputDirURL)
    
    if not os.path.exists(outputDirURL):
        os.mkdir(outputDirURL)
    
    inputFileURL = inputDirURL + options.input  +'.gpickle'
    outputFileURL = outputDirURL + options.input + '.gpickle'
    
    updateContourEdgeInfo(inputFileURL, outputFileURL)
    
    pass

if __name__ == '__main__':
    outputFileURL, inputFileURL = computeFeatures()
    pass