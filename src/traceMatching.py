'''
Created on Jul 20, 2015

@author: ash
'''

#global imports
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


# local imports
from roadSearch import featRoadLet


urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"


# generate the vehicle trae
def genCarTrace(_urlShpFile):
#     roadGraph = featRoadLet(_urlShpFile)
    roadGraph = nx.read_shp(urlShpFile)
    # use two random nodes in the graph and generate shortest path between them as vehicle trace
    
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
#     _source = random.choice(nodeList)
#     _target = random.choice(nodeList)
#     
#     _track = nx.astar_path(roadGraph, source = _source, target = _target)
#     
#     print _track
#     count = 0
#     while count < 20:
#         _node = random.choice(nodeList)
#         _track = []
#         
#         _track.append(_node)
#         _neighbors = nx.all_neighbors(roadGraph, _node)
#         count += 1
#         for _neighbor in _neighbors:
#             print _neighbor
#             pass
#         pass
#     
    
    
    count = 0
    _nodeLst = []
    while count < 100:
        _node = random.choice(nodeList)
        _edgeList = nx.to_edgelist(roadGraph, _node)
        count += 1
#         for _edge in _edgeList:
#             print count, _edge[0], _edge[1]
#             pass
        if len(_edgeList) > 1:
#             _neighbors = nx.all_neighbors(roadGraph, _node)
#             for _neighbor in _neighbors:
#                 print count, _neighbor
#                 pass
            _nodeLst.append(_node)
            pass
        pass
    print len(_nodeLst)
    print _nodeLst
    
    for _node in _nodeLst:
        _dfsTree = nx.dfs_tree(roadGraph, _node)
        print _dfsTree.edges()
        print '\n'
    
#     
#     _pathExistsFlag = False
#     _falseChoiceCount = 0
#     while not _pathExistsFlag:
#         _source = random.choice(nodeList)
#         _target = random.choice(nodeList)
#         _falseChoiceCount += 1
#         print _falseChoiceCount, _source, _target
#         
#         if nx.has_path(roadGraph, _source, _target):
#             _pathExistsFlag = True
#             pass
#         pass
#     
#     _track = nx.shortest_path(roadGraph,source=_source,target=_target)
#     
#     print _track
    
    pass


def randomWalk(urlShpFile):
    roadGraph = nx.read_shp(urlShpFile)
    # use two random nodes in the graph and generate shortest path between them as vehicle trace
    
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    trace = []
    
    _node = random.choice(nodeList)
    visited = []
    count = 0
    while count < 10:
        _neighbors = [x for x in nx.all_neighbors(roadGraph, _node)]
        _neighbor = random.choice(_neighbors)
        while _neighbor in visited:
            
            pass
        
        if not _neighbor in trace:
            trace.append(_neighbor)
            count += 1
            _node = _neighbor
        
    print trace
    
    pass

def neighorCount(urlShpFile):
    roadGraph = nx.read_shp(urlShpFile)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    print nNode
    edgesLst = nx.to_edgelist(roadGraph)
    print len(edgesLst)
    nodeNeighorhood = dict()
    neighborhoodSize = dict()
    nsize = [x for x in xrange(15)]
    for i in nsize:
        neighborhoodSize[i] = 0
        pass
    
    singleNeighborNodeList = []
    
    for _node in nodeList:
        _neighbors = [x for x in nx.all_neighbors(roadGraph, _node)]
        if not _node in nodeNeighorhood:
            nNeighbor = len(_neighbors)
            nodeNeighorhood[_node] = nNeighbor
            neighborhoodSize[nNeighbor] += 1
            pass
        pass
    
    for key in neighborhoodSize.keys():
        print key, '\t:', neighborhoodSize[key]
        pass    
    pass

def nodeNeighborScore(urlShpFile):
    roadGraph = nx.read_shp(urlShpFile)
    nodeList = roadGraph.nodes(data=False)
    
    pass


if __name__ == '__main__':
#     genCarTrace(urlShpFile)
#    randomWalk(urlShpFile)
    neighorCount(urlShpFile)
    pass