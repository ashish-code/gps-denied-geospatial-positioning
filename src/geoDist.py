'''
Created on Oct 29, 2015

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
from geopy.distance import great_circle

def getDistance(picklePathURL):
    roadGraph = nx.read_gpickle(picklePathURL)
    nodeList = roadGraph.nodes(data=False)
    edgeList = roadGraph.edges(data=False)
    _edgeList = roadGraph.edges(data=True)
    
    
    
    contourAttribute = nx.get_edge_attributes(roadGraph, 'contour')
    
    
#     print contourAttribute
    
    vincentyLengthList = []
    gcLengthList = []
    for edge in edgeList:
        node1 = edge[0]
        node2 = edge[1]
        _contour = contourAttribute[node1,node2]
        contourLength = len(_contour)
        edgeLengthVincenty = geopy.distance.vincenty(node1,node2).meters
        edgeLengthGC = geopy.distance.great_circle(node1,node2).meters
        
        vincentyLengthList.append(edgeLengthVincenty/contourLength)
        gcLengthList.append(edgeLengthGC/contourLength)
        
        pass
    
    print np.average(vincentyLengthList)
    print np.std(vincentyLengthList)
    print "------------------------------------------------------------"
    print np.average(gcLengthList)
    print np.std(gcLengthList)
    
    print 'nodes: ', len(nodeList)
    print 'edges: ', len(edgeList)
    
    
    nx.draw_networkx_edges(roadGraph, pos=dict(zip(roadGraph,nodeList)), edgelist=None, width=1, edge_color='b', alpha=0.2)
     
     
    plt.show()
    


if __name__ == '__main__':
    
    getDistance('/home/ash/Data/FixedShapeFiles/karlsruhe_germany_osm_roads/karlsruhe_germany_osm_roads.gpickle')
    pass