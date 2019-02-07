'''
Created on Jul 17, 2015

@author: ash
'''


import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np
import time
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
from scipy import interpolate
from scipy.interpolate import splprep, splev
from numpy import arange, cos, linspace, pi, sin

urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"

def genFeat(slpAngle, dictSize):
    if abs(slpAngle) > 360:
        slpAngle = slpAngle % 360
        pass
    
    if slpAngle < 0:
        slpAngle += 360
        pass
    
    binSize = 360 / dictSize
    feat = int(slpAngle // binSize)
    return feat
    pass

def getUniformSampledPoints(_r,_c):
    x = _r
    y = _c
    M = 1000
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    # ----------------------------------------------------------
    tol = 0.0004
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

def getRoadEdgeFeat(roadEdge):
    _edgePts = getEdgePoints(roadEdge)
    
    _r = [item[0] for item in _edgePts]
    _c = [item[1] for item in _edgePts]
    
    _r2,_c2 = getUniformSampledPoints(_r,_c)
    
    x = _r2
    y = _c2
    M = len(x)
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    z = t
    # spline parameters
    s=0.0 # smoothness parameter
    k=4 # spline order
    nest=-1 # estimate of number of knots needed (-1 = maximal)
    
    # find the knot points
    tckp,u = splprep([x,y,z],s=s,k=k,nest=-1)
    
    # evaluate spline, including interpolated points
    xnew,ynew,znew = splev(linspace(0,1,M),tckp)
    dx,dy,dz = splev(linspace(0,1,M), tckp, der=1)
    
    slp = []
    cnv = 180/ np.pi
    for i in xrange(len(dx)):
        if dx[i] == 0:
            if dy[i] < 0:
                slp.append(-90)
                pass
            else:
                slp.append(90)
                pass
            pass
        else:
            slp.append(np.arctan((dy[i]/dx[i]))*cnv )
            pass

    roadLetFeat = []
    for elem in slp:
        feat = genFeat(elem, 72)
        roadLetFeat.append(feat)
        pass
    
    return roadLetFeat
    pass

def featRoadLet(_urlShpFile):
    roadGraph = nx.read_shp(urlShpFile)
    roadEdgeList = nx.to_edgelist(roadGraph)
    nodeList = roadGraph.nodes(data=False)
    
    roadEdgeList = nx.to_edgelist(roadGraph)
    
    roadEdgeFeatDict = dict()
    for roadEdge in roadEdgeList:
        roadEdgeFeatDict[(roadEdge[0], roadEdge[1])] = getRoadEdgeFeat(roadEdge)
        pass
    nx.set_edge_attributes(roadGraph, 'roadLetCurvature', roadEdgeFeatDict)
    
    # return the road graph with the edge attribute with new value
    return roadGraph
    pass




  
if __name__ == '__main__':
    t0 = time.clock()
    featRoadLet(urlShpFile)
    timeElapsed = time.clock() - t0
    print timeElapsed
    pass