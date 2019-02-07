'''
Created on Nov 25, 2015

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
# 
# from geopy.distance import vincenty
from geopy import distance as geodist

_angleQuant = 1
_distQuant = 5

def analyzeRoadNetwork():
    parser = OptionParser()
<<<<<<< HEAD
    parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='washingtondc',help='input shapefile data directory')
=======
#     parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='karlsruhe_germany_osm_roads',help='input shapefile data directory')
    parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='washingtondc',help='input shapefile data directory')
    
>>>>>>> ca8ce07a6eda7e1d76d831d70fd73cf37a3d471f
    
    inputShpFileDir = '/home/ash/Data/InputShapeFiles/'
    fixedShpFileDir = '/home/ash/Data/FixedShapeFiles/'
    
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputDir = options.input
    
    dataDir = os.path.join(inputShpFileDir,inputDir)
    
    inputShpFileURL = dataDir + '/' + inputDir + '.shp'
    
    for fileName in os.listdir(dataDir):
        if fileName.endswith('shp'):
            fixedShpFileURL = os.path.join(fixedShpFileDir, inputDir, fileName)
            break
        pass
    
    pickledShpFileURL = os.path.join(fixedShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    

    roadGraph = nx.read_shp(inputShpFileURL, simplify=False)
    roadGraph = roadGraph.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    print 'nNode:',nNode
    edgeList = roadGraph.edges(data=True)
    nEdge = len(edgeList)
    print 'nEdge', nEdge
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(24,24))
    
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='k', alpha=0.3)
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, node_size=2, node_color='r', node_shape='.')
    plt.title('original')
    
    gcLengthList = []
    edgeSlopeList = []
    cnv = 360/ np.pi
    
    distAngleMat = np.zeros((1000/_distQuant, 180/_angleQuant))
    
    for edge in edgeList:
        node1 = edge[0]
        node2 = edge[1]
        edgeLengthGC = geodist.GreatCircleDistance(node1,node2).meters
        
        if edgeLengthGC > 999:
            edgeLengthGC = 999
            pass
        
        edgeLengthGC = int(edgeLengthGC/_distQuant)
        
        gcLengthList.append(edgeLengthGC)
        # ---------------------------------------------------------------------------------------------------
        dx = node1[0] - node2[0]
        if dx == 0:
            edgeSlope = 90/_angleQuant
            pass
        else:
            edgeSlope = int(np.arctan(np.abs( (node1[1] - node2[1]) / (dx) )) * cnv/_angleQuant)
        edgeSlopeList.append(edgeSlope)
        
        distAngleMat[edgeLengthGC, edgeSlope] += 1
        
        pass
    
    # for each node in the graph, find the angle between its edges that range from 0 to pi
    nodeAnglesList = []
    nodeList = roadGraph.nodes(data=False)
    for iNode, node in enumerate(nodeList):
        _neighborList = [_n for _n in nx.all_neighbors(roadGraph, node)]
        # compute angle between pairs of neighboring nodes
        if len(_neighborList) > 1:
            for _iNode, _nodeHead in enumerate(_neighborList):
                for _jNode, _nodeTail in enumerate(_neighborList):
                    edgeSlopeHead = 0
                    edgeSlopeTail = 0
                    if _iNode==_jNode:
                        pass
                    else:
                        dxHead = _nodeHead[0] - node[0]
                        if dxHead == 0:
                            edgeSlopeHead=90/_angleQuant
                            pass
                        else:
                            edgeSlopeHead = int(np.arctan(np.abs(_nodeHead[1]-node[1])/dxHead) * cnv/_angleQuant)
                            pass
                        # ----------------------------------------------------------------------------------------------------------
                        dxTail = _nodeTail[0] - node[0]
                        if dxTail == 0:
                            edgeSlopeTail=90/_angleQuant
                            pass
                        else:
                            edgeSlopeTail = int(np.arctan(np.abs(_nodeTail[1]-node[1])/dxTail) * cnv/_angleQuant)
                            pass
                    # ------------------------------------------------------------
                    # compute angle between the head and tail slopes, its should be between 0 and pi
                    _angle = np.abs(edgeSlopeTail - edgeSlopeHead)
                    if _angle > 180:
                        _angle = np.abs(180 - _angle)
                        pass
                    if _angle != 0:
                        nodeAnglesList.append(_angle)
                    pass
                pass
            pass
        pass
    
    # distance and angle apply to edges in edgeList, but it does not apply to angles at nodes and edges since edges and nodes in graph are different
    
    
    print np.average(gcLengthList)
    print np.std(gcLengthList)
    
    print np.average(edgeSlopeList)
    print np.std(edgeSlopeList)
    
    plt.figure(2, figsize=(24,12))
    plt.hist(gcLengthList, 200, log=True)
    
    plt.figure(3, figsize=(24,12))
    plt.hist(edgeSlopeList, 180/_angleQuant)
    
    
    from mpl_toolkits.mplot3d import Axes3D
    
    X = np.arange(1000/_distQuant)
    Y = np.arange(180/_angleQuant)
    X, Y = np.meshgrid(X, Y)
    
    Z = np.log(distAngleMat)
    Z = np.transpose(Z)
    
    print X.shape, Y.shape, Z.shape
    
    fig = plt.figure(4, figsize=(24,24))
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='hot', linewidth=0, antialiased=False)
#     ax.set_zlim(-1.01, 1.01)
    
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    plt.figure(5, figsize=(24,24))
    plt.matshow(Z, fignum=5)
    
    plt.figure(6, figsize=(12,12))
    plt.hist(nodeAnglesList, bins=180)
    
    plt.show()
    
    
    pass

if __name__ == '__main__':
    analyzeRoadNetwork()
    pass