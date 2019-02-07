'''
Created on Oct 28, 2015

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




def preProcess():
    
    parser = OptionParser()
    parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='karlsruhe_germany_osm_roads',help='input shapefile data directory')
    
    inputShpFileDir = '/home/ash/Data/InputShapeFiles/'
    fixedShpFileDir = '/home/ash/Data/FixedShapeFiles/'
    
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputDir = options.input
    
    dataDir = os.path.join(inputShpFileDir,inputDir)
    
    for fileName in os.listdir(dataDir):
        if fileName.endswith('shp'):
            fixedShpFileURL = os.path.join(fixedShpFileDir, inputDir, fileName)
            break
        pass
    
    pickledShpFileURL = os.path.join(fixedShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    
    return fixedShpFileURL, pickledShpFileURL
    pass

def printShpFileStats():
    
    fixedShpFileURL, pickledShpFileURL = preProcess()
    
    roadGraph = nx.read_shp(fixedShpFileURL, simplify=True)
    roadGraph = roadGraph.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    print 'nNode:',nNode
    edgeList = roadGraph.edges()
    nEdge = len(edgeList)
    print 'nEdge', nEdge
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(24,24))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b', alpha=0.3)
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, node_size=1, node_color='r', node_shape='.')
    plt.title('simplified')
    plt.show()
    
    # ------------------------------------------------------------------------------------------------
    roadGraph = nx.read_shp(fixedShpFileURL, simplify=False)
    roadGraph = roadGraph.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    print 'nNode:',nNode
    edgeList = roadGraph.edges()
    nEdge = len(edgeList)
    print 'nEdge', nEdge
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    plt.figure(2, figsize=(24,24))
    
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='k')
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, node_size=1, node_color='r', node_shape='.', alpha=0.3)
    plt.title('original')
    plt.show()
    
    
    
    
    pass
    


if __name__ == '__main__':
#     fixedShpFileURL, pickledShpFileURL = preProcess()
    printShpFileStats()
    pass