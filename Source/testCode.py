'''
Created on Nov 6, 2015

@author: gupta.637
'''


import networkx as nx
import matplotlib.pyplot as plt
import random
import pysal
from osm2nx import download_osm
from osm2nx import read_osm

osmFileURL = '/home/ash/Downloads/karlsruhe_kitti.osm'
# highway_cat = 'motorway|trunk|primary|secondary|tertiary|road|residential|service|motorway_link|trunk_link|primary_link|secondary_link|teriary_link'
highway_cat = 'motorway|trunk|primary|secondary|tertiary|road|residential|service|motorway_link|trunk_link|primary_link|secondary_link|teriary_link|path|footway|cycleway|motorway_junction|unclassified'


if __name__ == '__main__':
    
    osmFileURL = download_osm(8.325, 48.925, 8.50, 49.075, highway_cat)
    
    roadGraph = read_osm(osmFileURL, only_roads=True)
    roadGraph = roadGraph.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    edgeList = roadGraph.edges(data=False)
    nEdge = len(edgeList)
    pos = []
    for i in xrange(nNode):
        
        nodeDict = nodeList[i][1]
        nodePos = (nodeDict['lat'], nodeDict['lon'])
        pos.append(nodePos)
        
        pass
    print nNode, nEdge
    nodePos = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(20,20))
    nx.draw_networkx_edges(roadGraph, pos=nodePos, edgelist=None, width=1, edge_color='k')
    
#     _neighbors = nx.all_neighbors(roadGraph, nodeList[0])
    __neighbors = roadGraph.neighbors(_nodeList[0])
    print __neighbors
    plt.show()
    
    
    pass