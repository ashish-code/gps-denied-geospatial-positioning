'''
Created on Sep 9, 2015

@author: ash
'''


import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np
import os
import sys

from scipy import interpolate
from scipy.interpolate import interp1d, UnivariateSpline , splprep, splev
from numpy import arange, cos, linspace, pi, sin

#urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"
#graphPicklePath = "/home/ash/Data/fixedRoads/tl_2014_39049.gpickle"

# urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_11001_roads.shp"
# graphPicklePath = "/home/ash/Data/fixedRoads/tl_2014_11001.gpickle"


urlShpFile = "/home/ash/Data/fixedRoads/haiti_all_roads.shp"
graphPicklePath = "/home/ash/Data/fixedRoads/haiti_all_roads.gpickle"

_alphabetSize = 72

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

def updateContourEdgeInfo(urlShpFile, graphPicklePath):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    
    _edgeList = nx.to_edgelist(roadGraph)
    contourAttribute = dict()
    
    for _edge in _edgeList:
        _edgePts = getEdgePoints(_edge)
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
            slp.append(np.arctan((dy[i]/dx[i]))*cnv )
            pass
        
        # quantize the slope and assign to edge descriptor list
        roadLetFeat = []
        for elem in slp:
            feat = genFeat(elem, _alphabetSize)
            roadLetFeat.append(feat)
            pass
        
        # the roadEdgefeature into a new attribute
        # -----------------------------------------------------------
        # DEBUG:
        print _edge[0],_edge[1], roadLetFeat
        # -----------------------------------------------------------
        
        contourAttribute[(_edge[0],_edge[1])] = roadLetFeat
        pass
    
    nx.set_edge_attributes(roadGraph, 'contour', contourAttribute)
    nx.write_gpickle(roadGraph, graphPicklePath)
    pass

def checkContourAttribute(graphPicklePath):
    roadGraph = nx.read_gpickle(graphPicklePath)
    edgeList = nx.generate_edgelist(roadGraph, data=['contour'])
    
    roadGraph = roadGraph.to_undirected()
    print "graph has been read"
    nodeList = roadGraph.nodes(data=True)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(roadGraph,pos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraph))
    print "number of edges: " + str(nx.number_of_edges(roadGraph))
#     nx.draw_networkx(roadGraph, pos=shpLayout, with_labels=False, node_size = 1, edge_color="r")
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='r')
    plt.show()
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, nodelist=None, node_size=1, node_color='b', node_shape='d')
    plt.show()

    
    for i, edge in enumerate(edgeList):
        print i, edge

if __name__ == '__main__':
    if os.path.exists(graphPicklePath) == False:
        updateContourEdgeInfo(urlShpFile, graphPicklePath)
    checkContourAttribute(graphPicklePath)
    pass