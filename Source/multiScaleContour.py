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
parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='karlsruhe_germany_osm_roads',help='input shapefile data directory')

inputShpFileDir = '/home/ash/Data/InputShapeFiles/'
fixedShpFileDir = '/home/ash/Data/MultiScaleShapeFiles/'

_alphabetSize = 72
binSize = int(360 / _alphabetSize)

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
# 
# def getUniformSampledPoints(_r,_c):
#     x = _r
#     y = _c
#     
#     M = 5000
#     t = np.linspace(0, len(x), M)
#     x = np.interp(t, np.arange(len(x)), x)
#     y = np.interp(t, np.arange(len(y)), y)
#     # ----------------------------------------------------------
# #     tol = 0.0004
#     tol = 0.00001
#     # ----------------------------------------------------------
#     approxEdgeLength = math.sqrt((x[0]-x[-1])**2 + (y[0]-y[-1])**2)
#     
#     if approxEdgeLength/5 < tol:
#         tol = approxEdgeLength/5
#         pass
# 
#     i, idx = 0, [0]
#     while i < len(x):
#         total_dist = 0
#         for j in range(i+1, len(x)):
#             total_dist += math.sqrt((x[j]-x[j-1])**2 + (y[j]-y[j-1])**2)
#             if total_dist > tol:
#                 idx.append(j)
#                 break
#         i = j+1
#     idx.append(len(x)-1)
#     xn = x[idx]
#     yn = y[idx]
#     return xn,yn


def getMeasuredSamplePoints(_r,_c, d):
    x = _r
    y = _c
    
    
    _dist = 0
    for i in xrange(len(x)-1):
        node1 = (x[i],y[i])
        node2 = (x[i+1],y[i+1])
        _dist += geopy.distance.vincenty(node1,node2).meters
        _dist += geopy.distance.great_circle(node1,node2).meters
        pass
    
    M = int(_dist/2*d)
    
    if M < len(x):
        M = len(x)
    
    t = np.linspace(0, len(x), M)
    x = np.interp(t, np.arange(len(x)), x)
    y = np.interp(t, np.arange(len(y)), y)
    return x,y


def genFeat(slpAngle, dictSize):
    slpAngle = slpAngle % 360
#     if abs(slpAngle) > 360:
#         slpAngle = slpAngle % 360
#         pass
    
    if slpAngle < 0:
        slpAngle += 360
        pass
    
    slpAngle = abs(slpAngle)
    if math.isnan(slpAngle):
        slpAngle = 0
    feat = int(slpAngle // binSize)
    return feat
    pass

def updateContourEdgeInfo(urlShpFile, graphPicklePath):
    roadGraphd = nx.read_shp(urlShpFile)
    roadGraph = roadGraphd.to_undirected()
    
    _edgeList = nx.to_edgelist(roadGraph)
    contourAttribute = dict()
    
    # spline parameters
    s=0.0 # smoothness parameter
    k=4 # spline order
    
    samplingDists = [1,5,10,20]
    
    for d in samplingDists:
    
        for _edge in _edgeList:
            _edgePts = getEdgePoints(_edge)
            _r = [item[0] for item in _edgePts]
            _c = [item[1] for item in _edgePts]
            
            _r2,_c2 = getMeasuredSamplePoints(_r,_c,d)
            
            x = _r2
            y = _c2
    #         M = 2*( len(x) + k)
            M = len(x)
            
            if M <= k:
                M = 2*k
            
            t = np.linspace(0, len(x), M)
            x = np.interp(t, np.arange(len(x)), x)
            y = np.interp(t, np.arange(len(y)), y)
            z = t
            
            # find the knot points
            tckp,u = splprep([x,y,z],s=s,k=k,nest=-1)
            
            # evaluate spline, including interpolated points
            xnew,ynew,znew = splev(linspace(0,1,M),tckp)
            dx,dy,dz = splev(linspace(0,1,M), tckp, der=1)
            
            slp = []
            cnv = 180/ np.pi
            for i in xrange(len(dx)):
                slp.append(np.arctan((dy[i]/dx[i]))*cnv )
                pass
            
            # quantize the slope and assign to edge descriptor list
            roadLetFeat = []
            for elem in slp:
                feat = genFeat(elem, _alphabetSize)
                roadLetFeat.append(feat)
                pass
            
            contourAttribute[(_edge[0],_edge[1])] = roadLetFeat
            pass
        
        contourName = 'contour' + str(d)
        nx.set_edge_attributes(roadGraph, contourName, contourAttribute)
        
        
    nx.write_gpickle(roadGraph, graphPicklePath)
    pass


def snap_verts(shp,tolerance=10000000,arc=True):
    """
    snap_verts -- Snap verts that are within tolerance meters of each other.

    Description -- Snapping should be performed with a very small tolerance.
                   The goal is not to change the network, but to ensure rounding
                   errors don't prevent edges from being split at proper intersections.
                   The default of 1mm should be adequate if the input is of decent quality.
                   Higher snapping values can be used to correct digitizing errors, but care
                   should be taken.

    Arguments
    ---------
    tolerance -- float -- snapping tolerance in meters
    arc -- bool -- If true, Ard Distance will be used instead of Euclidean

    Returns
    -------
    generator -- each element is a new pysal.cg.Chain with corrected vertices.
    """
    kmtol = tolerance/1000.

    data = np.concatenate([rec.vertices for rec in shp])
    
    if arc:
        kd = pysal.cg.KDTree(data,distance_metric="Arc",radius = pysal.cg.sphere.RADIUS_EARTH_KM)
    else:
        kd = pysal.cg.KDTree(data)
    q = kd.query_ball_tree(kd,kmtol)
    ### Next three lines assert that snappings are mutual... if 1 snaps to 8, 8 must snap to 1.
    for r,a in enumerate(q):
        for o in a:
            assert a==q[o]
    ### non-mutual snapping can happen.
    ### consider the three points, A (-1,0), B (0,0), C (1,0) and a snapping tolerance of 1.
    ### A-> B
    ### B-> A,C
    ### C-> B
    ### For now, try lowering adjusting the tolerance to avoid this.

    data2 = np.empty_like(data)
    for i,r in enumerate(q):
        data2[i] = data[r].mean(0)
    pos=0
    for rec in shp:
        vrts = rec.vertices
        n = len(vrts)
        nrec = pysal.cg.Chain(map(tuple,data2[pos:pos+n]))
        pos+=n
        yield nrec
    
def find_nodes(shp):
    """
    find_nodes -- Finds vertices in a line type shapefile that appear more than once and/or are end points of a line

    Arguments
    ---------
    shp -- Shapefile Object -- Should be of type Line.

    Returns
    -------
    set
    """
    node_count = {}
    for road in shp:
        vrts = road.vertices
        for node in vrts:
            if node not in node_count:
                node_count[node] = 0
            node_count[node] += 1
        node_count[vrts[0]] += 1
        node_count[vrts[-1]] += 1
    return set([node for node,c in node_count.iteritems() if c > 1])

def split_at_nodes(shp):
    """
    split_at_nodes -- Split line features at nodes

    Arguments
    ---------
    shp -- list or shapefile -- Chain features to be split at common nodes.

    Returns
    -------
    generator -- yields pysal.cg.Chain objects
    """
    nodes = find_nodes(shp)
    nodeIds = list(nodes)
    nodeIds.sort()
    nodeIds = dict([(node,i) for i,node in enumerate(nodeIds)])
    
    for road in shp:
        vrts = road.vertices
        midVrts = set(road.vertices[1:-1]) #we know end points are nodes
        midNodes = midVrts.intersection(nodes) # find any nodes in the middle of the feature.
        midIdx = [vrts.index(node) for node in midNodes] # Get their indices
        midIdx.sort()
        if midIdx:
            #print vrts
            starts = [0]+midIdx
            stops = [x+1 for x in midIdx]+[None]
            for start,stop in zip(starts,stops):
                feat = pysal.cg.Chain(vrts[start:stop])
                rec = (nodeIds[feat.vertices[0]],nodeIds[feat.vertices[-1]],False)
                yield feat,rec
        else:
            rec = (nodeIds[road.vertices[0]],nodeIds[road.vertices[-1]],False)
            yield road,rec




def createSpatialNetworkShapefile(inshp,outshp):
    assert inshp.lower().endswith('.shp')
    assert outshp.lower().endswith('.shp')
    shp = pysal.open(inshp,'r')
    snapped = list(snap_verts(shp,.001))
    o = pysal.open(outshp,'w')
    odb = pysal.open(outshp[:-4]+'.dbf','w')
    odb.header = ["FNODE","TNODE","ONEWAY"]
    odb.field_spec = [('N',20,0),('N',20,0),('L',1,0)]

    new = list(split_at_nodes(snapped))
    for feat,rec in new:
        o.write(feat)
        odb.write(rec)
    o.close()
    odb.close()
    print "Split %d roads in %d network edges"%(len(shp),len(new))


def createSpatialNetworkPickleFile(inPickle,outPickle):
    assert inPickle.lower().endswith('.gpickle')
    assert outPickle.lower().endswith('.gpickle')
    shp = nx.read_gpickle(inPickle)
    snapped = list(snap_verts(shp,.001))
    o = pysal.open(outPickle,'w')
    odb = pysal.open(outPickle[:-4]+'.dbf','w')
    odb.header = ["FNODE","TNODE","ONEWAY"]
    odb.field_spec = [('N',20,0),('N',20,0),('L',1,0)]

    new = list(split_at_nodes(snapped))
    for feat,rec in new:
        o.write(feat)
        odb.write(rec)
    o.close()
    odb.close()
    print "Split %d roads in %d network edges"%(len(shp),len(new))


def preProcess():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputDir = options.input
    
    dataDir = os.path.join(inputShpFileDir,inputDir)
    inputShpFileURL = ''
    for fileName in os.listdir(dataDir):
        if fileName.endswith('shp'):
            inputShpFileURL = os.path.join(dataDir,fileName)
            fixedShpFileURL = os.path.join(fixedShpFileDir, inputDir, fileName)
            break
        pass
    
    fixedDir = os.path.join(fixedShpFileDir, inputDir)
    
    if os.path.exists(fixedDir) == False:
        os.mkdir(fixedDir)
        pass
    
    if os.path.exists(inputShpFileURL) and not os.path.exists(fixedShpFileURL):
        createSpatialNetworkShapefile(inputShpFileURL,fixedShpFileURL)
    
#     inputPickledShpFileURL = os.path.join(inputShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    fixedPickledShpFileURL = os.path.join(fixedShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    
    
    if os.path.exists(inputShpFileURL) and not os.path.exists(fixedPickledShpFileURL):
        updateContourEdgeInfo(fixedShpFileURL, fixedPickledShpFileURL)
        
    
    return fixedShpFileURL, fixedPickledShpFileURL
    pass


def computeFeatures():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    inputDir = options.input
    
    dataDir = os.path.join(inputShpFileDir,inputDir)
    inputShpFileURL = ''
    for fileName in os.listdir(dataDir):
        if fileName.endswith('shp'):
            inputShpFileURL = os.path.join(dataDir,fileName)
            fixedShpFileURL = os.path.join(fixedShpFileDir, inputDir, fileName)
            break
        pass
    
    fixedDir = os.path.join(fixedShpFileDir, inputDir)
    
    if os.path.exists(fixedDir) == False:
        os.mkdir(fixedDir)
        pass
    
    if os.path.exists(inputShpFileURL):
        createSpatialNetworkShapefile(inputShpFileURL,fixedShpFileURL)
    
#     inputPickledShpFileURL = os.path.join(inputShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    fixedPickledShpFileURL = os.path.join(fixedShpFileDir, inputDir, (fileName[:-4] + '.gpickle'))
    
    
    if os.path.exists(inputShpFileURL):
        updateContourEdgeInfo(fixedShpFileURL, fixedPickledShpFileURL)
        
    
    return fixedShpFileURL, fixedPickledShpFileURL
    pass


def displayShpFile(pickledShpFileURL):
    roadGraph = nx.read_gpickle(pickledShpFileURL)
    nodeList = roadGraph.nodes(data=True)
    _nodeList = roadGraph.nodes(data=False)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    shpLayout = dict(zip(roadGraph,pos))
    plt.figure(1, figsize=(12,12))
    nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
    plt.show()
    
    
    pass
if __name__ == '__main__':
    fixedShpFileURL, pickledShpFileURL = computeFeatures()
    displayShpFile(pickledShpFileURL)
