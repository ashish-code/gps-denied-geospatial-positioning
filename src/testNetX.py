'''
Created on Jun 21, 2015

@author: ash
'''

import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np
from sets import Set
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
from scipy import interpolate
from scipy.interpolate import splprep, splev
from numpy import arange, cos, linspace, pi, sin

# urlShpFile = "/home/ash/Data/tl_2014_39049_roads/tl_2014_39049_roads.shp"
outShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"
# urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"

urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_11001_roads.shp"

# urlShpFile = "/home/ash/Data/fixedRoads/haiti_all_roads.shp"
# urlShpFile = "/home/ash/Data/haiti_all_roads/Haiti_all_roads.shp"


def readShpFile(_urlShpFile):
    roadGraph = nx.read_shp(_urlShpFile)
    roadGraph = roadGraph.to_undirected()
    print "graph has been read"
    nodeList = roadGraph.nodes(data=True)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(roadGraph,pos))
    print "number of nodes: " + str(nx.number_of_nodes(roadGraph))
    print "number of edges: " + str(nx.number_of_edges(roadGraph))
#     nx.draw_networkx(roadGraph, pos=shpLayout, with_labels=False, node_size = 1, edge_color="r")
    plt.figure(1, figsize=(24,24))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b', alpha=0.3)
    plt.title("Transport Network Graph Representation")
    nx.draw_networkx_nodes(roadGraph, pos=shpLayout, nodelist=None, node_size=3, node_color='k', node_shape='.')
#     plt.title("Washington DC")
    plt.show()
#     print len(nodeList)
#     for i in xrange(10):
#         print nodeList[i]
#         print nodeList[i][0]
#         print nodeList[i][0][0]
#         print nodeList[i][0][1]
#         
    pass

def builtInGraph():
    petersen = nx.petersen_graph()
    nx.draw(petersen)
    plt.show()
    pass

def incidenceMat():
    roadGraph = nx.read_shp(urlShpFile)
    imat = nx.incidence_matrix(roadGraph)
    print imat
    pass

def adjGraph():
    roadGraph = nx.read_shp(urlShpFile)
    adjMat = nx.adjacency_matrix(roadGraph)
    print adjMat
    pass

def edjList():
    roadGraph = nx.read_shp(urlShpFile)
    roadEdgeList = nx.to_edgelist(roadGraph)
    for rEdge in roadEdgeList:
        print rEdge
    pass

def roadTree():
    roadGraph = nx.read_shp(urlShpFile)
    rnodelist = roadGraph.nodes(data=False)
    for i in xrange(5):
        print rnodelist[i]
    rbfsTree = nx.dfs_tree(roadGraph, rnodelist[772])
    nodeList = rbfsTree.nodes(data=True)
    nNode = len(nodeList)
    print nNode
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(rbfsTree,pos))
    nx.draw_networkx_nodes(rbfsTree, pos=shpLayout, edgelist=None, width=1, edge_color='r')
    plt.show()
    pass

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

def dispEdgeGraph(edgePoints):
    edgeGraph = nx.Graph()
    nNode = len(edgePoints)
    nIdx = range(0, nNode)
    pos = dict(zip(nIdx, edgePoints))
    print pos
    
    edgeGraph.add_nodes_from(pos.keys())
    for n, p in pos.iteritems():
        edgeGraph.node[n]['pos'] = p
        pass
    
    for n in xrange(nNode-1):
        edgeGraph.add_edge(n, n+1)
        pass
    
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx(edgeGraph, pos, node_size=9, node_shape='o', alpha=0.25,node_color='r', edge_width=1, edge_color='b', font_size=8, with_labels=False)
    plt.show()
    
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
#         print pos
        
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


def roadData():
    roadGraph = nx.read_shp(urlShpFile)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    qnodeList = []
    for i in xrange(1):
        qnodeList.append(random.choice(nodeList))
        pass
    print 'qnodelist', qnodeList
    roadEdgeList = nx.to_edgelist(roadGraph, qnodeList)
    _flag = 10
    for roadEdge in roadEdgeList:
        if _flag:
            edgePointPairLst = getEdgePoints(roadEdge)
            
            nAlpha = [float(x) for x in roadEdge[0]]
            nOmega = [float(x) for x in roadEdge[1]]
#             print nAlpha
#             print nOmega
#             for edgePointPair in edgePointPairLst:
#                 print edgePointPair
#                 pass
            edgePoints = []
            edgePoints.append(nAlpha)
            for edgePointPair in edgePointPairLst:
                edgePoints.append(edgePointPair)
                pass
            edgePoints.append(nOmega)
            

            dispEdgeGraph(edgePoints)
            
            _flag -= 1
            pass
        
    pass


def roadData2():
    roadGraph = nx.read_shp(urlShpFile)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    _node = random.choice(nodeList)
    _nodeEdgeList = nx.edges(roadGraph, _node)
    count = 0
    while (len(_nodeEdgeList) < 2):
        _node = random.choice(nodeList)
        _nodeEdgeList = nx.edges(roadGraph, _node)
        count += 1
    
    
    print _nodeEdgeList
    _nodeEdgeLst = nx.to_edgelist(roadGraph, _node)
    for _edge in _nodeEdgeLst:
        _edgePts = getEdgePoints(_edge)
        print _edgePts
    
    edgePointsLst = []
    for roadEdge in _nodeEdgeLst:
        
        edgePointPairLst = getEdgePoints(roadEdge)
        edgePointsLst.append(edgePointPairLst)
        
    
    dispNodeEdgeGraph(edgePointsLst)        
    pass


def roadData1():
    roadGraph = nx.read_shp(urlShpFile)
    roadEdgeList = nx.to_edgelist(roadGraph)
    _flag = True
    for roadEdge in roadEdgeList:
        while _flag:
            print roadEdge[0]
            print roadEdge[1]
            roadEdgeData = roadEdge[2]
            edgeKeyList = roadEdgeData.keys()
            for edgeKey in edgeKeyList:
                print edgeKey 
                print roadEdgeData[edgeKey]
                pass
            edgePointStr = roadEdgeData['Wkt']
            b1 = edgePointStr.find('(')
            b2 = edgePointStr.find(')')
            edgePointsSubStr = edgePointStr[b1+1:b2]
            print edgePointsSubStr
            
            edgePointPairStrLst = edgePointsSubStr.split(',')
            edgePointPairLst = []
            for edgePointPairStr in edgePointPairStrLst:
                edgePointPair = [float(x) for x in edgePointPairStr.split(' ')]
                edgePointPairLst.append(edgePointPair)
                pass
            
#             coordinates of points in an edge
            for _item in edgePointPairLst:
                print _item
                pass
            _flag = False
    pass

def UpdateEdgeList():
    roadGraph = nx.read_shp(urlShpFile)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    qnodeList = []
    for i in xrange(1):
        qnodeList.append(random.choice(nodeList))
        pass
    
    roadEdgeList = nx.to_edgelist(roadGraph, qnodeList)
    for roadEdge in roadEdgeList:
        edgePointPairLst = getEdgePoints(roadEdge)
        print edgePointPairLst
    pass

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def getEdgeLength(roadEdge):
    edgePointPairLst = getEdgePoints(roadEdge)
    totalDist = 0
    for i in xrange(len(edgePointPairLst)-1):
        totalDist += distance(edgePointPairLst[i], edgePointPairLst[i+1])
        pass
    pass
    return totalDist

def findSamplingDist():
    roadGraph = nx.read_shp(urlShpFile)
#     roadEdgeList = nx.generate_edgelist(roadGraph, delimiter=',', data=False)
    roadEdgeList = nx.to_edgelist(roadGraph)

    
    lengthAttribute = dict()
    for roadEdge in roadEdgeList:
        lengthAttribute[(roadEdge[0], roadEdge[1])] = getEdgeLength(roadEdge)
        pass
    
    nx.set_edge_attributes(roadGraph, 'roadLength', lengthAttribute)
    
    roadLengthAttribute = nx.get_edge_attributes(roadGraph, 'roadLength')
    roadLengths = []
    for roadEdge in roadEdgeList:
        roadLength = roadLengthAttribute[(roadEdge[0], roadEdge[1])]
        roadLengths.append(roadLength)
        
        pass
    print 'number of edges ', len(roadEdgeList)
    print 'total edge length ', sum(roadLengths)
    print 'average edge length ', np.mean(roadLengths)
    print 'std deviation edge length', np.std(roadLengths)
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

def interpEdge():
    # second attempt at interpolation of edge
    roadGraph = nx.read_shp(urlShpFile)
    roadEdgeList = nx.to_edgelist(roadGraph)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    _node = random.choice(nodeList)
    _nodeEdgeList = nx.edges(roadGraph, _node)
    count = 0
    while (len(_nodeEdgeList) < 4):
        _node = random.choice(nodeList)
        _nodeEdgeList = nx.edges(roadGraph, _node)
        count += 1
    
    _nodeEdgeLst = nx.to_edgelist(roadGraph, _node)
    print _nodeEdgeLst
    
    # ------------------------------------------------------
    
    edgePointsLst = []
    for roadEdge in _nodeEdgeLst:
        edgePointPairLst = getEdgePoints(roadEdge)
        edgePointsLst.append(edgePointPairLst)
         
    
    
    # ------------------------------------------------------
    
    interPtsLst =[]
    figCount = 1
    
    
    figCount += 1
    ax = plt.figure(figCount, figsize=(12,12))
    for _edge in _nodeEdgeLst:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
        _r2,_c2 = getUniformSampledPoints(_r,_c)
        
        x = _r2
        y = _c2
        M = len(x)
        t = np.linspace(0, len(x), M)
        x = np.interp(t, np.arange(len(x)), x)
        y = np.interp(t, np.arange(len(y)), y)
        z = t
        # spline parameters
        s=0.0 # smoothness parameter
        k=4 # spline order
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
        
        
        
        plt.plot(x, y, 'kd', label='data point')

        for X, Y, Z in zip(x, y, roadLetFeat):
            # Annotate the points 5 _points_ above and to the left of the vertex
            plt.annotate('{}'.format(Z), xy=(X,Y), xytext=(-.5, .5), color='blue', ha='right',textcoords='offset points')
        
        
#         data,=plt.plot(x,y,'.',label='data')
        fit,=plt.plot(xnew,ynew,'r-',label='spline')
        #plt.legend()
        plt.xlabel('x')
        plt.ylabel('y')
        plt.hold(True)
    plt.show()
    
    
    dispNodeEdgeGraph(edgePointsLst,2)
    
    dispNodeEdgeGraphClean(edgePointsLst, 3)
    
    
        
#         _r2mid = _r2[::2]
#         tck = interpolate.splrep(_r2, _c2, s=0)
#         
#         _c2mid_der = interpolate.splev(_r2mid, tck, der=1)
#         
#         figCount += 1
#         plt.figure(figCount, figsize=(12,12))
#         plt.plot(_r2mid, _c2mid_der)
        
    
    
    
    
    
    pass


def interpolateEdge():
    roadGraph = nx.read_shp(urlShpFile)
    roadEdgeList = nx.to_edgelist(roadGraph)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    _node = random.choice(nodeList)
    _nodeEdgeList = nx.edges(roadGraph, _node)
    count = 0
    while (len(_nodeEdgeList) < 3):
        _node = random.choice(nodeList)
        _nodeEdgeList = nx.edges(roadGraph, _node)
        count += 1
    
    _nodeEdgeLst = nx.to_edgelist(roadGraph, _node)
    
    # ------------------------------------------------------
    
    edgePointsLst = []
    for roadEdge in _nodeEdgeLst:
        edgePointPairLst = getEdgePoints(roadEdge)
        edgePointsLst.append(edgePointPairLst)
         
    dispNodeEdgeGraph(edgePointsLst,2)
    
    # ------------------------------------------------------
    
    interPtsLst =[]
    for _edge in _nodeEdgeLst:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
        
        _r2,_c2 = getUniformSampledPoints(_r,_c)
#         print _r2, _c2
        
        # ------------------------------------------------------
#         if len(_r2) > 3:
#             f = interp1d(_r2,_c2, kind='cubic')
#         else:
#             f = interp1d(_r2,_c2, kind='linear')
#             pass
        # ------------------------------------------------------
        
        if len(_r2) > 3:
            _k = 3
            pass
        else:
            _k = 1
            pass
        
        
        f = UnivariateSpline(_r2, _c2, k=_k, s=0)
        
        
        _r3 = _r2
        _c3 = f(_r3)

        _interPts = [(_r3[i], _c3[i]) for i in xrange(len(_r3))]
        
        
#         fd = f.derivative().root()
#         print fd

        interPtsLst.append(_interPts)
        pass
    
    
    dispNodeEdgeGraph(interPtsLst,1)
#     x = [_r3[i] for i in xrange(len(_r3))]
#     y  = [_c3[i] for i in xrange(len(_c3))]
#  
#     tck, u = interpolate.splprep([x,y])
#     
#     M = 1000
#     t = np.linspace(0, len(_r), M)
#     dxdt, dydt = interpolate.splev(t,tck,der=1)
#     print dxdt, dydt
#     _derPts = [(x[i],y[i]) for i in xrange(len(x))]
#     dispNodeEdgeGraph(_derPts, 2)
    
    
    
    
#     x = _r
#     y = _c
#     M = 1000
#     t = np.linspace(0, len(x), M)
#     x = np.interp(t, np.arange(len(x)), x)
#     y = np.interp(t, np.arange(len(y)), y)
#     tol = 0.001
#     i, idx = 0, [0]
#     while i < len(x):
#         total_dist = 0
#         for j in range(i+1, len(x)):
#             total_dist += math.sqrt((x[j]-x[j-1])**2 + (y[j]-y[j-1])**2)
#             if total_dist > tol:
#                 idx.append(j)
#                 break
#         i = j+1
# 
#     xn = x[idx]
#     yn = y[idx]
#     fig, ax = plt.subplots()
#     ax.plot(x, y, '-')
#     ax.scatter(xn, yn, s=50)
#     plt.show()
    
    
    
    
#     x = np.arange(0,2*np.pi+np.pi/4,2*np.pi/8)
#     y = np.sin(x)
#     tck = interpolate.splrep(x,y,s=0)
#     print tck
#     xnew = np.arange(0,2*np.pi,np.pi/50)
#     ynew = interpolate.splev(xnew,tck,der=0)
#     
#     plt.figure()
#     plt.plot(x,y,'x',xnew,ynew,xnew,np.sin(xnew),x,y,'b')
#     plt.legend(['Linear','Cubic Spline', 'True'])
#     plt.axis([-0.05,6.33,-1.05,1.05])
#     plt.title('Cubic-spline interpolation')
#     plt.show()
    
    pass

def accessEdges(urlShpFile):
    roadGraphD = nx.read_shp(urlShpFile)
    roadGraph = roadGraphD.to_undirected()
    nodeList = nx.nodes(roadGraph)
    node1 = random.choice(nodeList)
    node2 = list(nx.all_neighbors(roadGraph, node1))[0]
    
    print node1, node2
    edgeList = roadGraph.edges(node1, data=True)
    for edge in edgeList:
        if edge[1] == node2:
            print edge
    pass

def checkEdgeList(urlShpFile):
    roadGraphD = nx.read_shp(urlShpFile)
    roadGraph = roadGraphD.to_undirected()
    edgeList = roadGraph.edges(data = False, default = None)
    nodeList = nx.nodes(roadGraph)
    
    for i in xrange(10):
        node1 = random.choice(nodeList)
        
        edges = nx.edges(roadGraph, node1)
        print node1
        for edge in edges:
            print edge
            pass
        print "-----------------------------------------"
    
    pass

def plotRoadDesc(urlShpFile):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    _nodeListd = roadGraph.nodes(data=True)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(_nodeListd[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    
    _node = random.choice(nodeList)
    _nodeEdgeList = nx.edges(roadGraph, _node)
    count = 0
    while (len(_nodeEdgeList) < 3):
        _node = random.choice(nodeList)
        _nodeEdgeList = nx.edges(roadGraph, _node)
        count += 1
        pass
    
    _nodeList = Set([_node])
    maxLevel = 7
    levelCount = 0
    while levelCount < maxLevel:
        _nodeList_ = Set(_nodeList)
        for __node in _nodeList:
            _neighbors = nx.all_neighbors(roadGraph, __node)
            for _node_ in _neighbors:
                _nodeList_.add(_node_)
                pass
            levelCount += 1
            pass
        for node1 in _nodeList_:
            _nodeList.add(node1)
        pass
    print _nodeList
    nNodeList = []
    for node1 in _nodeList:
        nNodeList.append(node1)
        pass
    
    nGraph = nx.subgraph(roadGraph, nNodeList)
    nEdgeList = nx.to_edgelist(roadGraph, nNodeList)
    
    ax = plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=2, edge_color='b', alpha=0.3)
    for _edge in nEdgeList:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
        _r2,_c2 = getUniformSampledPoints(_r,_c)
        
        x = _r2
        y = _c2
        M = len(x)
        t = np.linspace(0, len(x), M)
        x = np.interp(t, np.arange(len(x)), x)
        y = np.interp(t, np.arange(len(y)), y)
        z = t
        # spline parameters
        s=0.0 # smoothness parameter
        k=4 # spline order
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
        
        
        
        plt.plot(x, y, 'k.')

        for X, Y, Z in zip(x, y, roadLetFeat):
            # Annotate the points 5 _points_ above and to the left of the vertex
            plt.annotate('{}'.format(Z), xy=(X,Y), xytext=(-.5, .5), color='black', ha='right',textcoords='offset points')
        
        
#         data,=plt.plot(x,y,'.',label='data')
        fit,=plt.plot(xnew,ynew,'r-', alpha=0.5, linewidth=4)
        #plt.legend()
        plt.xlabel('x')
        plt.ylabel('y')
        plt.hold(True)
    plt.show()
    
        
def plotPathDesc(urlShpFile):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    _nodeListd = roadGraph.nodes(data=True)
    nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(_nodeListd[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    
    _node1 = random.choice(nodeList)
    _node2 = random.choice(nodeList)
    
    _path = nx.astar_path(roadGraph, _node1, _node2, None)
    _pathedges = zip(_path[0:-1], _path[1:])
    
    nEdgeList = []
    for i, node1 in enumerate(_path[0:-1]):
        edges = roadGraph.edges(node1, data=True)
        for edge in edges:
            if edge[1] == _path[i+1]:
                nEdgeList.append(edge)
        pass
    
    
    termnodelist = [_node1,_node2]
    
    
    
    
    
    ax1 = plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=2, edge_color='b', alpha=0.3)
    for _edge in nEdgeList:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
        _r2,_c2 = getUniformSampledPoints(_r,_c)
        
        x = _r2
        y = _c2
        M = len(x)
        t = np.linspace(0, len(x), M)
        x = np.interp(t, np.arange(len(x)), x)
        y = np.interp(t, np.arange(len(y)), y)
        z = t
        # spline parameters
        s=0.0 # smoothness parameter
        k=4 # spline order
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
        
        
        
        plt.plot(x, y, 'k.')

        for X, Y, Z in zip(x, y, roadLetFeat):
            # Annotate the points 5 _points_ above and to the left of the vertex
            plt.annotate('{}'.format(Z), xy=(X,Y), xytext=(-.5, .5), color='black', ha='right',textcoords='offset points')
        
        
#         data,=plt.plot(x,y,'.',label='data')
        plt.plot(xnew,ynew,'r-', alpha=0.5, linewidth=4)
        
        plt.xlabel('x')
        plt.ylabel('y')
        plt.hold(True)
        pass
    
    
    
    
    ax2 = plt.figure(2, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=2, edge_color='b', alpha=0.3)
    for _edge in nEdgeList:
        _edgePts = getEdgePoints(_edge)
        _r = [item[0] for item in _edgePts]
        _c = [item[1] for item in _edgePts]
        
        _r2,_c2 = getUniformSampledPoints(_r,_c)
        
        x = _r2
        y = _c2
        M = len(x)
        t = np.linspace(0, len(x), M)
        x = np.interp(t, np.arange(len(x)), x)
        y = np.interp(t, np.arange(len(y)), y)
        z = t
        # spline parameters
        s=0.0 # smoothness parameter
        k=4 # spline order
        nest=-1 # estimate of number of knots needed (-1 = maximal)
        
        # find the knot points
        tckp,u = splprep([x,y,z],s=s,k=k,nest=-1)
        
        # evaluate spline, including interpolated points
        xnew,ynew,znew = splev(linspace(0,1,M),tckp)
#         dx,dy,dz = splev(linspace(0,1,M), tckp, der=1)
#         
#         slp = []
#         cnv = 180/ np.pi
#         for i in xrange(len(dx)):
#             slp.append(np.arctan((dy[i]/dx[i]))*cnv )
#         print slp
#         
#         
#         # -------------------
#         # quantize the slope and assign to edge descriptor list
#         
#         roadLetFeat = []
#         for elem in slp:
#             feat = genFeat(elem, 72)
#             roadLetFeat.append(feat)
        
        
        
#         plt.plot(x, y, 'k.')
# 
#         for X, Y, Z in zip(x, y, roadLetFeat):
#             # Annotate the points 5 _points_ above and to the left of the vertex
#             plt.annotate('{}'.format(Z), xy=(X,Y), xytext=(-.5, .5), color='black', ha='right',textcoords='offset points')
        
        
#         data,=plt.plot(x,y,'.',label='data')
        plt.plot(xnew,ynew,'r-', alpha=0.5, linewidth=4)
        
        plt.xlabel('x')
        plt.ylabel('y')
        plt.hold(True)
        pass
    
    
    
    
    plt.show()
    
    pass
def shpFileStats():
    
    pass
    

if __name__ == '__main__':
#     builtInGraph()
    readShpFile(outShpFile)
#     readShpFile(outShpFile)
#     incidenceMat()
#     adjGraph()
#     edjList()
#     roadTree()
#     roadData1()
#     roadData2()
#     UpdateEdgeList()
#     findSamplingDist()
#     interpolateEdge()
#     interpEdge()
#     accessEdges(urlShpFile)
#     checkEdgeList(urlShpFile)
#     plotRoadDesc(urlShpFile)
#     plotPathDesc(urlShpFile)
#     shpFileStats()
    
    pass