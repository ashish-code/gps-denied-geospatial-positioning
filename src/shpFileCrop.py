'''
Created on Oct 29, 2015

@author: ash
'''

'''
crop map shapefile based on lat, long extents
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

def findExtent(nodeList):
    minLongi = nodeList[0][0]
    maxLongi = nodeList[0][0]
    minLat = nodeList[0][1]
    maxLat = nodeList[0][1]
    for node in nodeList:
        longi = node[0]
        lat = node[1]
        if minLongi > longi: minLongi = longi
        if maxLongi < longi: maxLongi = longi
        if minLat > lat: minLat = lat
        if maxLat < lat: maxLat = lat
        
        pass
    
    return minLongi, maxLongi, minLat, maxLat
    pass


def cropShpFile():
    parser = OptionParser()
    parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='/home/ash/Data/FixedShapeFiles/karlsruheroads/karlsruhe_germany_osm_roads.shp',help='input shapefile url')
    parser.add_option('-o','--output',type='string',metavar='output',dest='output',default='/home/ash/Data/InputShapeFiles/karlsruhecityroads/karlsruhe_germany_osm_roads.gpickle', help='ouput shapefile url')
    parser.add_option('-s', '--shpDir',type='string',metavar='shpDir',dest='shpDir',default='/home/ash/Data/InputShapeFiles/karlsruhecityroads/', help='output shapefile directory')
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    fixedShpFileURL = options.input
    cropShpFileURL = options.output
    shpFileDir = options.shpDir
    
    roadGraph = nx.read_shp(fixedShpFileURL)
    roadGraph = roadGraph.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    
    print _nodeList[0]
    
    minLong, maxLong, minLat, maxLat = findExtent(_nodeList)
    print minLong, maxLong, minLat, maxLat
    
    minLongNew = minLong + 0.33*(maxLong-minLong)
    maxLongNew = minLong + 0.67*(maxLong-minLong)
    
    minLatNew = minLat + 0.33*(maxLat-minLat)
    maxLatNew = minLat + 0.67*(maxLat-minLat)
    
    print minLongNew, maxLongNew, minLatNew, maxLatNew
    
    minLongNew = 8.325
    maxLongNew = 8.500
    minLatNew = 48.925
    maxLatNew = 49.075
    
    nodeListNew = []
    for node in _nodeList:
        longi = node[0]
        lat = node[1]
        if longi > minLongNew and longi < maxLongNew and lat > minLatNew and lat < maxLatNew:
            nodeListNew.append(node)
            pass
        pass
    
    print len(nodeListNew)
#     roadGraphCrop = nx.subgraph(roadGraph, nodeListNew)
    roadGraphCrop = roadGraph.subgraph(nodeListNew)
    
    
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraph))
    print "number of edges: " + str(nx.number_of_edges(roadGraph))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b', alpha=0.2)
    
    
    cropNodeList = roadGraphCrop.nodes(data=True)
    ncropNode = len(cropNodeList)
    croppos = []
    for i in xrange(ncropNode):
        croppos.append(cropNodeList[i][0])
        pass
    cropShpLayout = dict(zip(roadGraphCrop, croppos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraphCrop))
    print "number of edges: " + str(nx.number_of_edges(roadGraphCrop))
    nx.draw_networkx_edges(roadGraphCrop, pos=cropShpLayout, edgelist=None, width=1, edge_color='r')
    
    nx.write_gpickle(roadGraphCrop, path=cropShpFileURL)
    nx.write_shp(roadGraphCrop, shpFileDir)
    
    # rename the output shapefiles in the shpFileDir
    fileName = cropShpFileURL.split('/')[-1]
    fileName = fileName.split('.')[0]
    
    os.chdir(shpFileDir)
    os.rename('edges.shp', fileName + '.shp')
    os.rename('edges.dbf', fileName + '.dbf')
    os.rename('edges.shx', fileName + '.shx')
    
    plt.show()
    
    

if __name__ == '__main__':
    cropShpFile()
    pass