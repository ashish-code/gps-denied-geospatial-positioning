'''
Created on Aug 31, 2015

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
urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"

def roadNetStats(_urlShpFile):
    roadGraphd = nx.read_shp(_urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    print "graph has been read"
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
    
    neighborCount = {}
    for node in _nodeList:
        neighbors = list(nx.all_neighbors(roadGraph, node))
        n = len(neighbors)
        
        if neighborCount.has_key(n):
            neighborCount[n] += 1
            pass
        else:
            neighborCount[n] = 1
        pass
    
    for item in neighborCount.keys():
        print item, ":", neighborCount[item]
    pass

def rndRoad(_urlShpFile):
    roadGraphd = nx.read_shp(_urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    print "graph has been read"
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(roadGraph,pos))
    
    _node1 = random.choice(_nodeList)
    _node2 = random.choice(_nodeList)
    
#     paths = list(nx.all_simple_paths(roadGraph, _node1, _node2))
#     print len(paths)
#     _path = paths[0]
    
    #_path = nx.astar_path(roadGraph, _node1, _node2)
    _path = nx.dijkstra_path(roadGraph, source=_node1, target=_node2)
    
    _pathedges = nx.to_edgelist(roadGraph, nodelist=_path)
    termnodelist = [_node1,_node2]
    
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    nx.draw_networkx(roadGraph, pos=shpLayout, nodelist=_path, edgelist = _pathedges, node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, nodelist=termnodelist, node_size=60, node_color='k', node_shape='o')
    plt.show()
    
    pass

def rndTraversal(_urlShpFile):
    roadGraphd = nx.read_shp(_urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    _node1 = random.choice(_nodeList)
    _node2 = random.choice(_nodeList)
    
    _path = nx.astar_path(roadGraph, _node1, _node2, None)
    _pathedges = zip(_path[0:-1], _path[1:])
    
    termnodelist = [_node1,_node2]
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    nx.draw_networkx(roadGraph, pos=shpLayout, nodelist=_path, edgelist = _pathedges, node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, nodelist=termnodelist, node_size=60, node_color='k', node_shape='o')
    plt.show()
    pass

def genFeat(slpAngle, dictSize):
    if abs(slpAngle) > 360:
        slpAngle = slpAngle % 360
        pass
    
    if slpAngle < 0:
        slpAngle += 360
        pass
    
    binSize = 360 / dictSize
    feat = int(slpAngle // binSize)
    return feat
    pass

def dispNodeEdgeGraph(edgePointsLst, figNo =1):
    plt.figure(figNo, figsize=(12,12))
    for edgePoints in edgePointsLst:
        edgeGraph = nx.Graph()
        nNode = len(edgePoints)
        nIdx = range(0, nNode)
        pos = dict(zip(nIdx, edgePoints))

        edgeGraph.add_nodes_from(pos.keys())
        for n, p in pos.iteritems():
            edgeGraph.node[n]['pos'] = p
            pass
        
        for n in xrange(nNode-1):
            edgeGraph.add_edge(n, n+1)
            pass
        nx.draw_networkx(edgeGraph, pos, node_size=9, node_shape='o', alpha=0.25,node_color='r', edge_width=1, edge_color='b', font_size=8, with_labels=False)
    
    plt.show()

    pass

def dispNodeEdgeGraphClean(edgePointsLst, figNo =1):
    plt.figure(figNo, figsize=(12,12))
    for edgePoints in edgePointsLst:
        edgeGraph = nx.Graph()
        nNode = len(edgePoints)
        nIdx = range(0, nNode)
        pos = dict(zip(nIdx, edgePoints))
        
        edgeGraph.add_nodes_from(pos.keys())
        for n, p in pos.iteritems():
            edgeGraph.node[n]['pos'] = p
            pass
        
        for n in xrange(nNode-1):
            edgeGraph.add_edge(n, n+1)
            pass

        nx.draw_networkx(edgeGraph, pos, node_size=0, node_shape='o', alpha=0.25,node_color='k', edge_width=5, edge_color='k', font_size=8, with_labels=False)
    
    plt.show()

    pass

def getUniformSampledPoints(_r,_c):
    x = _r
    y = _c
    M = 1000
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    # ----------------------------------------------------------
    tol = 0.0004
    # ----------------------------------------------------------
    approxEdgeLength = math.sqrt((x[0]-x[-1])**2 + (y[0]-y[-1])**2)
    
    if approxEdgeLength/5 < tol:
        tol = approxEdgeLength/5
        pass
    
    i, idx = 0, [0]
    while i < len(x):
        total_dist = 0
        for j in range(i+1, len(x)):
            total_dist += math.sqrt((x[j]-x[j-1])**2 + (y[j]-y[j-1])**2)
            if total_dist > tol:
                idx.append(j)
                break
        i = j+1
    idx.append(len(x)-1)
    xn = x[idx]
    yn = y[idx]
    return xn,yn

def getEdgePoints(roadEdge):
    roadEdgeData = roadEdge[2]
    edgeKeyList = roadEdgeData.keys()
    edgePointStr = roadEdgeData['Wkt']
    b1 = edgePointStr.find('(')
    b2 = edgePointStr.find(')')
    edgePointsSubStr = edgePointStr[b1+1:b2]
    
    edgePointPairStrLst = edgePointsSubStr.split(',')
    edgePointPairLst = []
    for edgePointPairStr in edgePointPairStrLst:
        edgePointPair = [float(x) for x in edgePointPairStr.split(' ')]
        edgePointPairLst.append(edgePointPair)
        pass
    
    return edgePointPairLst
    pass

def rndTrajectory(_urlShpFile):
    roadGraphd = nx.read_shp(_urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    _node1 = random.choice(_nodeList)
    _node2 = random.choice(_nodeList)
    
    _path = nx.astar_path(roadGraph, _node1, _node2, None)
    _pathedges = zip(_path[0:-1], _path[1:])
    
    pathedges = []
    for i, node1 in enumerate(_path[0:-1]):
        edges = roadGraph.edges(node1, data=True)
        for edge in edges:
            if edge[1] == _path[i+1]:
                pathedges.append(edge)
        pass
    
    
    termnodelist = [_node1,_node2]
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    nx.draw_networkx(roadGraph, pos=shpLayout, nodelist=_path, edgelist = _pathedges, node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, nodelist=termnodelist, node_size=60, node_color='k', node_shape='o')
#     plt.show()
    
    _edgePts = []
    for roadEdge in pathedges:
        edgePointPairLst = getEdgePoints(roadEdge)
        _edgePts.extend(edgePointPairLst)
        
    
    interPtsLst =[]
    print _edgePts
    
    _r = [item[0] for item in _edgePts]
    _c = [item[1] for item in _edgePts]
    
    _r2,_c2 = getUniformSampledPoints(_r,_c)
    
    
    x = _r2
    y = _c2
#     M = len(x)
    M = 10*len(x)
    
    print 'M', M
    
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    z = t
    # spline parameters
    s=0 # smoothness parameter
    k=5 # spline order
    nest=-1 # estimate of number of knots needed (-1 = maximal)
    
    # find the knot points
    tckp,u = splprep([x,y,z],s=s,k=k,nest=-1)
    
    # evaluate spline, including interpolated points
    xnew,ynew,znew = splev(linspace(0,1,M),tckp)
    dx,dy,dz = splev(linspace(0,1,M), tckp, der=1)
    
    slp = []
    cnv = 180/ np.pi
    for i in xrange(len(dx)):
        slp.append(np.arctan((dy[i]/dx[i]))*cnv )
    print slp
    
    
    # -------------------
    # quantize the slope and assign to edge descriptor list
    
    roadLetFeat = []
    for elem in slp:
        feat = genFeat(elem, 72)
        roadLetFeat.append(feat)

#     plt.plot(x, y, 'kd', label='data point')

#     for X, Y, Z in zip(x, y, roadLetFeat):
#         # Annotate the points 5 _points_ above and to the left of the vertex
#         plt.annotate('{}'.format(Z), xy=(X,Y), xytext=(-.5, .5), color='blue', ha='right',textcoords='offset points')
    fig = plt.figure(2, figsize=(12,12))
    ax = plt.plot(xnew,ynew,'k-',label='Trajectory', linewidth=3)
    #plt.legend()
    
    #ax.tick_params(axis='both', which='both', bottom='off', top='off', labelbottom='off', right='off', left='off', labelleft='off')
    
    plt.xticks([])
    plt.xlabel("")
    plt.yticks([])
    plt.ylabel("")
    
#     plt.xlabel('x')
#     plt.ylabel('y')
#     plt.hold(True)
    
    plt.show()
    
#     dispNodeEdgeGraph(_edgePts,2)
#     dispNodeEdgeGraphClean(_edgePts, 3)
    pass

def mysubgraph(_urlShpFile):
    roadGraphd = nx.read_shp(_urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    nodeList = roadGraph.nodes(data=True)
    
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    _node1 = random.choice(_nodeList)
    _node2 = random.choice(_nodeList)
    _path = nx.astar_path(roadGraph, _node1, _node2, None)
    
    subG = nx.subgraph(roadGraph, _path)
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx(subG, pos=shpLayout, edgelist=None, node_size=40, node_color='r', node_shape='d', edgewidth=10, edge_color='r', with_labels=False)
    plt.show()
    

if __name__ == '__main__':
#     roadNetStats(urlShpFile)
#     rndRoad(urlShpFile)
#     rndTraversal(urlShpFile)
    
    rndTrajectory(urlShpFile)
#     mysubgraph(urlShpFile)
    pass