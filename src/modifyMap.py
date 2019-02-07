'''
Created on Nov 4, 2015

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
parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='karlsruhe_germany_osm_roads',help='input shapefile data directory')

fixedShpFileDir = '/home/ash/Data/FixedShapeFiles/'
denseShpFileDir = '/home/ash/Data/DenseShapeFiles/'

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

def getMeasuredSamplePoints(_r,_c):
    x = _r
    y = _c
    
    
#     _dist = 0.0
#     for i in xrange(len(x)-1):
#         node1 = (x[i],y[i])
#         node2 = (x[i+1],y[i+1])
#         _dist += geopy.distance.vincenty(node1,node2).meters
#         pass
    
    crowDist = geopy.distance.vincenty((x[0],y[0]), (x[-1],y[-1])).meters
#     M = int(np.ceil(_dist/5))
    M = int(np.ceil(crowDist))
    
    if M < len(x):
        M = len(x)
    
    
    
    t = np.linspace(0, len(x), M)
    _x = np.interp(t, np.arange(len(x)), x)
    _y = np.interp(t, np.arange(len(y)), y)
    
    # prune out points that are identical
    pts = zip(_x,_y)
    pts = list(set(pts))
    
#     print _dist, len(x),M, len(pts)
    
    return pts




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



def splitnadd():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputName = options.input
    
    fixedShpFileURL = fixedShpFileDir + inputName + '/' + inputName + '.shp'
    denseShpFileURL = denseShpFileDir + inputName + '/' + inputName + '.shp'
    
    fixedPickleFileURL = fixedShpFileDir + inputName + '/' + inputName + '.gpickle'
    densePickleFileURL = denseShpFileDir + inputName + '/' + inputName + '.gpickle'
    
    roadGraph = nx.read_gpickle(fixedPickleFileURL)
    
    edgeList = roadGraph.edges(data=True)
    _edgeList = nx.to_edgelist(roadGraph)
    
    
#     count =0
    nnodeList = []
    distList = []
    for _edge in _edgeList:
        
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        _nodesLst = getMeasuredSamplePoints(_r,_c)
# #         _nodesLst = zip(_r2,_c2)
#         print _nodesLst
#         for i in xrange(len(_nodesLst)-1):
#             node1 = _nodesLst[i]
#             node2 = _nodesLst[i+1]
#             _dist = geopy.distance.vincenty(node1,node2).meters
#             print _dist,
#         print '---------------------------------'
#         plt.figure(i, figsize=(28,28))
#         plt.scatter([x[0] for x in _nodesLst], [x[1] for x in _nodesLst], s=20, c = 'k', marker='x')
#         plt.scatter(_r,_c,s=30,c='r',marker='o')
#         plt.show()
# #         # add nodes at each point in _r2,_c2
# #         print _edgePts[0], _edgePts[-1]
# #         print [float(x) for x in _edgePts[0]], [float(x) for x in _edgePts[-1]]
# #         print _r2[0],_c2[0], _r2[-1],_c2[-1]
# #         print _edgePts[0][0] - _r2[0], _edgePts[0][1] - _c2[0], _edgePts[-1][0] - _r2[-1], _edgePts[-1][1] - _c2[-1]
# #         
# #         print '---------------------------------'
#         count += 1
#         if count > 10: break
        
        
        # remove original edge
        _dist = 0.0
        for i in xrange(len(_nodesLst)-1):
            node1 = _nodesLst[i]
            node2 = _nodesLst[i+1]
            _dist += geopy.distance.vincenty(node1,node2).meters
            pass
                   
        nNode = len(_nodesLst)
        nnodeList.append(nNode)
        distList.append(_dist)
        nmid = int(len(_nodesLst)/2)
        crowDist = geopy.distance.vincenty(_nodesLst[0],_nodesLst[nmid]).meters + geopy.distance.vincenty(_nodesLst[nmid],_nodesLst[-1]).meters
        print nNode, _dist, crowDist, len(_edgePts), geopy.distance.vincenty(_nodesLst[0],_nodesLst[-1]).meters
    
    print np.average(nnodeList), np.average(distList), np.average([x/y for (x,y) in zip(distList,nnodeList)])
    
    pass

if __name__ == '__main__':
    splitnadd()
    pass