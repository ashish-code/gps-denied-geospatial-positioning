'''
Created on Aug 21, 2015

@author: ash
'''


import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np

from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
from scipy import interpolate
from scipy.interpolate import splprep, splev
from numpy import arange, cos, linspace, pi, sin

# urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"
urlShpFile = "/home/ash/Data/tl_2013_06_prisecroads/tl_2013_06_prisecroads.shp"
outShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"

def findmst(roadGraph, node):
    mst = nx.minimum_spanning_edges(roadGraph)
    edgeList = list(mst)
    tnodes = roadGraph.nodes(data=False)
    shpLayout = dict(zip(tnodes,tnodes))
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos = shpLayout, edgelist = edgeList )
    plt.show()
    pass

def findTree(roadGraph, node):
    T = nx.bfs_tree(roadGraph, node)
#     T = nx.minimum_spanning_tree(roadGraph)
    
#     print node, T.edges()
    tnodes = T.nodes(data=False)
    shpLayout = dict(zip(tnodes,tnodes))
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx(T, pos=shpLayout, node_size=1, node_shape='d', alpha=0.25,node_color='r', edge_width=1, edge_color='b', with_labels=False)
    plt.show()
    pass

def parseGraph(urlShpFile):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
#     roadGraph = nx.read_shp(urlShpFile)
    
#     roadGraph = list(nx.connected_component_subgraphs(roadGraph.to_undirected()))[0]
    
    
    nodeLst = roadGraph.nodes(data=False)
    roadEdgeList = nx.to_edgelist(roadGraph)
    
    
    
    weightAttribute = dict()
    for roadEdge in roadEdgeList:
        weightAttribute[(roadEdge[0], roadEdge[1])] = 1
        pass
    
    nx.set_edge_attributes(roadGraph, 'weight', weightAttribute)
    
    
    for node in nodeLst:
        _n = roadGraph.neighbors(node)
        print len(_n), node
        if(len(_n) > 4):
            findTree(roadGraph, node)
#             findmst(roadGraph, node)
    pass
    


if __name__ == '__main__':
    parseGraph(outShpFile)
    pass