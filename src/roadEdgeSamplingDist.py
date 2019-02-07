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
import geopy
from geopy.distance import vincenty

parser = OptionParser()
parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='karlsruhe_germany_osm_roads',help='input shapefile data directory')

inputShpFileDir = '/home/ash/Data/InputShapeFiles/'
fixedShpFileDir = '/home/ash/Data/FixedShapeFiles/'

_alphabetSize = 72
binSize = int(360 / _alphabetSize)


def getEdgePoints(roadEdge):
    print roadEdge
    
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


def analyseGraphDist(graphPicklePath):
    roadGraph = nx.read_gpickle(graphPicklePath)
    
    nodeList = roadGraph.nodes(data=True)
    
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    
    _node1 = random.choice(_nodeList)
    _node2 = random.choice(_nodeList)
    _path = nx.dijkstra_path(roadGraph, _node1, _node2, None)
    
    pathGraph = nx.subgraph(roadGraph, _path)
    termnodelist = [_node1,_node2]
    
    edgeList = nx.to_edgelist(roadGraph, nodelist=None)
    pathEdgeList = nx.to_edgelist(pathGraph, nodelist = None)
    
    contourAttribute = nx.get_edge_attributes(roadGraph, 'contour')
    pathContourAttribute = nx.get_edge_attributes(pathGraph, 'contour')
    
    
    distList = []
    
    for pedge in pathEdgeList:
        pathEdgeFeat = pathContourAttribute[(pedge[0],pedge[1])]
#         print pathEdgeFeat
#         print len(pathEdgeFeat)
        edgePtsList = getEdgePoints(pedge)
#         print edgePtsList
        
        _distList = []
        
        for i in range(len(edgePtsList)-1):
            _dist = geopy.distance.vincenty(edgePtsList[i],edgePtsList[i+1]).meters
            _distList.append(_dist)
            pass
        
        distList.append(np.sum(_distList)/len(edgePtsList))
    
    
    
    print np.average(distList)
    print np.std(distList)
    
def displayShpFile(pickledShpFileURL):
    roadGraph = nx.read_gpickle(pickledShpFileURL)
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    plt.show()    

if __name__ == '__main__':
    
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputDir = options.input
    
    graphPicklePath = fixedShpFileDir + inputDir + '/' + inputDir + '.gpickle'
    print graphPicklePath
    analyseGraphDist(graphPicklePath)
    displayShpFile(graphPicklePath)
    pass