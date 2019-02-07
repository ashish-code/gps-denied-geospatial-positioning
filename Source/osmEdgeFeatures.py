'''
Created on Nov 20, 2015

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
parser.add_option('-i', '--input', type='string', metavar='input', dest='input', default='karlsruheOSM', help='input osm file data directory name')
parser.add_option('-t','--fileType', type='string', metavar='fileType', dest='fileType', default='gpickle', help='input osm data file extension type')

inputFileDir = '/home/ash/Data/InputShapeFiles/'

_alphabetSize = 72
binSize = int(360 / _alphabetSize)

def analyseOSMGraph():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    
    inputDirURL = inputFileDir + options.input + '/'
    
    if not os.path.exists(inputDirURL):
        os.mkdir(inputDirURL)
    
    inputFileURL = inputDirURL + options.input  + '.' + options.fileType
    roadGraph = nx.read_gpickle(inputFileURL)
    
    edgeList = roadGraph.edges(data = True)
    nodeList = roadGraph.nodes(data = True)
    
    nNode = len(nodeList)
    nEdge = len(edgeList)
    
    pos = []
    for i in xrange(nNode):
        
        nodeDict = nodeList[i][1]
        nodePos = (nodeDict['lat'], nodeDict['lon'])
        pos.append(nodePos)
        pass
    
    print nNode, nEdge
    inode = random.choice(nodeList)
    print inode
    
    iedge = random.choice(edgeList)
    print iedge
    
    nodePos = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(20,20))
#     nx.draw_networkx_edges(roadGraph, pos=nodePos, edgelist=None, width=1, edge_color='k')
    nx.draw_networkx_nodes(roadGraph, pos=nodePos, node_size=1)
    plt.show()
    
    pass

if __name__ == '__main__':
    analyseOSMGraph()
    pass