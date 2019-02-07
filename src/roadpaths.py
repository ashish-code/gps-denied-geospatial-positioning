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

urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"
outShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"



def parseGraph(urlShpFile):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    
    roadEdgeList = nx.to_edgelist(roadGraph)
    weightAttribute = dict()
    for roadEdge in roadEdgeList:
        weightAttribute[(roadEdge[0], roadEdge[1])] = 1
        pass
    nx.set_edge_attributes(roadGraph, 'weight', weightAttribute)
    
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(roadGraph,pos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraph))
    print "number of edges: " + str(nx.number_of_edges(roadGraph))
    
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    
    _nodeLst = []
    for node in _nodeList:
        if len(nx.neighbors(roadGraph,node)) > 3:
            _nodeLst.append(node)
    
    _node1 = random.choice(_nodeLst)
    
    
    T = nx.dfs_tree(roadGraph, source=_node1)
    
    tnodeList = T.nodes()
    
    
    _node2 = random.choice(tnodeList)
    
#     _node2 = tnodeList[-1]
#     _path = nx.all_simple_paths(roadGraph, source=_node1, target=_node2, cutoff=None)
    _path = nx.astar_path(roadGraph, source=_node1, target=_node2)
    
    _pathedges = nx.to_edgelist(roadGraph, nodelist=_path)
#     print _pathedges
    termnodelist = [_node1,_node2]
    
    nx.draw_networkx(T, pos=dict(zip(T,tnodeList)), nodelist=_path, edgelist = _pathedges, node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    nx.draw_networkx_nodes(T, pos=dict(zip(T,tnodeList)), nodelist=termnodelist, node_size=60, node_color='k', node_shape='o')
    print _node1, _node2, _path
    plt.show()
    pass

def twotrees(urlShpFile):
#     roadGraphd = nx.read_shp(urlShpFile)
#     roadGraph = roadGraphd.to_undirected()
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraphud = roadGraphd.to_undirected()
    roadGraph = nx.minimum_spanning_tree(roadGraphud)
    
    
    
    
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    _nodeLst = []
    for node in _nodeList:
        if len(nx.neighbors(roadGraph,node)) > 3:
            _nodeLst.append(node)
    
    _node1 = random.choice(_nodeLst)
    
    mstsucc = nx.bfs_successors(roadGraph, source = _node1)
    
    for i, key in enumerate(mstsucc.keys()):
        print i, key, mstsucc[key]
    
    
    dfsT = nx.dfs_tree(roadGraph, source=_node1)
    bfsT = nx.bfs_tree(roadGraph, source = _node1)
    
    print bfsT.nodes()
    print bfsT.adjacency_list()
    print nx.bfs_successors(bfsT, source=_node1)
    
    plt.figure(1, figsize=(12,12))
    
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(roadGraph,pos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraph))
    print "number of edges: " + str(nx.number_of_edges(roadGraph))
    
    
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    nx.draw_networkx_nodes(dfsT, pos=dict(zip(dfsT,dfsT.nodes())), nodelist=[_node1], node_size=60, node_color='k', node_shape='o')
    nx.draw_networkx(dfsT, pos=dict(zip(dfsT,dfsT.nodes())), nodelist = dfsT.nodes(), node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)

    
    plt.figure(2, figsize=(12,12))
    nx.draw_networkx(bfsT, pos=dict(zip(bfsT,bfsT.nodes())), nodelist=bfsT.nodes(), node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    nx.draw_networkx_nodes(bfsT, pos=dict(zip(bfsT,bfsT.nodes())), nodelist=[_node1], node_size=60, node_color='k', node_shape='o')
    plt.show()
    pass


if __name__ == '__main__':
#     parseGraph(urlShpFile)
    twotrees(urlShpFile)
    