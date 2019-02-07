'''
Created on Aug 18, 2015

@author: ash
'''

import nx_spatial as ns
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

import sets

urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"

# def tryNxSpatial():
#     roadGraph = ns.read_shp(urlShpFile)
#     nodeList = roadGraph.nodes(data=False)
#     edgeList = roadGraph.edges()
#     
#     nodeDict = {}
#     nodeDictRev = {}
#     
#     for i, node in enumerate(nodeList):
#         nodeDict[i] = node
#         nodeDictRev[node] = i
#     
#     _nodeList = sets.Set()
#     nodeCount = {}
#     nodeCountList = []
#     for edge in edgeList:
#         nodeCountList.append(edge[0])
#         nodeCountList.append(edge[1])
#         _nodeList.add(edge[0])
#         _nodeList.add(edge[1])
#         pass
#     
#     for node in _nodeList:
#         count = 0
#         for _n in nodeCountList:
#             if(node == _n):
#                 count += 1
#                 pass
#             pass
#         nodeCount[node] = count
#         print count, node[0], node[1]
    
    # plot the nodes with 1, 2 , ... adjacent neighbors
    



# nsizelist = []
# for node in nodeList:
#     _nlist = roadGraph.neighbors(node)
#     nsizelist.append(len(_nlist))
#     print node, len(_nlist)
#     pass

def plotNodes():
    roadGraph = nx.read_shp(urlShpFile)
    nodeLst = roadGraph.nodes(data=False)
    neighborCount = []
    list0 = []
    list1 = []
    list2 = []
    list3 = []
    list4 = []
    for node in nodeLst:
        _n = roadGraph.neighbors(node)
        n = len(_n)
        neighborCount.append(n)
        print n, node
        if n == 0:
            list0.append(node)
        if n == 1:
            list1.append(node)
        elif n == 2:
            list2.append(node)
        elif n == 3:
            list3.append(node)
        elif n == 4:
            list4.append(node)
        else:
            pass
        pass
    
    shpLayout0 = dict(zip(list0,list0))
    shpLayout1 = dict(zip(list1,list1))
    shpLayout2 = dict(zip(list2,list2))
    shpLayout3 = dict(zip(list3,list3))
    shpLayout4 = dict(zip(list4,list4))
    
    plt.figure(1, figsize=(12,12))

    nx.draw_networkx_nodes(roadGraph, pos=shpLayout0, nodelist=list0, node_size=1, node_color='c', node_shape='o')
    plt.show()
    
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout1, nodelist=list1, node_size=2, node_color='y', node_shape='d')
#     plt.show()
    
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout2, nodelist=list2, node_size=3, node_color='k', node_shape='d')
#     plt.show()
    
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout3, nodelist=list3, node_size=4, node_color='r', node_shape='d')
#     plt.show()
    
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout4, nodelist=list4, node_size=5, node_color='g', node_shape='d')
    plt.show()
    
if __name__ == '__main__':
    plotNodes()
    pass
