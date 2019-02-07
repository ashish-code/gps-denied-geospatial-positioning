'''
Created on Aug 27, 2015

@author: ash
'''

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import shapefile

# urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"
urlShpFile = "/home/ash/Data/tl_2013_06_prisecroads/tl_2013_06_prisecroads.shp"

def synGraph():
    G = nx.Graph()
    N = 50
    for i in xrange(N):
        G.add_node(i)
        pass
    
    count = 0
    while(count < N):
        a = count
        b = np.random.randint(N)
        if a != b:
            G.add_edge(a, b)
            count += 1
            pass
        pass
    
    plt.figure(1)
    nx.draw_networkx(G)
#     plt.show()
    
    dfsT = nx.bfs_tree(G, source=10)
    plt.figure(2)
    nx.draw_networkx(dfsT)
    plt.show()
    
    pass

def findTree(roadGraph, node, shpLayout):
    T = nx.dfs_tree(roadGraph, node)
#     T = nx.minimum_spanning_tree(roadGraph)
    
#     print node, T.edges()
    tnodes = T.nodes(data=False)
#     shpLayout = dict(zip(tnodes,tnodes))
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx(T, pos=shpLayout, node_size=1, node_shape='d', alpha=0.25,node_color='r', edge_width=1, edge_color='b', with_labels=False)
    plt.show()
    pass

def syn2graph():
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    _pos = []
    for i in xrange(nNode):
        _pos.append(nodeList[i][0])
        pass
    
    nodedict = dict(zip(_nodeList,range(nNode)))
    
    G = nx.Graph()
    for i in range(nNode):
        G.add_node(i)
        pass
    
    edgeList = nx.to_edgelist(roadGraph)
    for _edge in edgeList:
        node1 = _edge[0]
        node2 = _edge[1]
        node1idx = nodedict[node1]
        node2idx = nodedict[node2]
        G.add_edge(node1idx, node2idx)
        pass
    shpLayout = dict(zip(G,_pos))
    plt.figure(1)
    nx.draw_networkx(G, pos=shpLayout, node_size=1, edge_width=1, with_labels=False)
    plt.show()
    nodeLst = G.nodes(data=False)
    for node in nodeLst:
        _n = G.neighbors(node)
        print len(_n), node
        if(len(_n) > 3):
            findTree(G, node, shpLayout)
    pass
    
    
    
    pass

def trypyshp():
    shpf = shapefile.Reader('/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads')
    
    pass


if __name__ == '__main__':
#     synGraph()
    syn2graph()
#     trypyshp()
    pass