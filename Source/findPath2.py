'''
Created on Mar 13, 2016

@author: gupta.637
'''

import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import numpy as np
import os
import sys

from scipy import interpolate
from scipy.interpolate import interp1d, UnivariateSpline , splprep, splev
from numpy import arange, cos, linspace, pi, sin
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from optparse import OptionParser
import difflib
from matplotlib.lines import lineStyles
# homeDir = '/Users/gupta.637'
homeDir = "/home/ash/"
parser = OptionParser()
parser.add_option('-i','--input',type='string',metavar='input',dest='input',default='columbus',help='input shapefile data directory')
import matplotlib.colors as clrs
import matplotlib


roomShpFile = homeDir+'/Data/indoorMaps/osuSmithLabFloor02/065-02Clean.shp'

# urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_39049_roads.shp"
# graphPicklePath = "/home/ash/Data/fixedRoads/tl_2014_39049.gpickle"
# pltTitle = "Columbus OH, USA"

# urlShpFile = homeDir+"/Data/fixedRoads/tl_2014_39049_roads.shp"
# graphPicklePath = homeDir+"/Data/fixedRoads/tl_2014_39049.gpickle"
# pltTitle = "Columbus OH, USA"


# urlShpFile = "/home/ash/Data/fixedRoads/tl_2014_11001_roads.shp"
# graphPicklePath = "/home/ash/Data/fixedRoads/tl_2014_11001.gpickle"

# urlShpFile = "/home/ash/Data/FixedShapeFiles/montpellierosmroads/montpellier.osm-roads.shp"
# graphPicklePath = "/home/ash/Data/FixedShapeFiles/montpellierosmroads/montpellier.osm-roads.gpickle"
# pltTitle = "Montpellier, France"


# urlShpFile = "/home/ash/Data/FixedShapeFiles/paris_france_osm_roads/paris_france_osm_roads.shp"
# graphPicklePath = "/home/ash/Data/FixedShapeFiles/paris_france_osm_roads/paris_france_osm_roads.gpickle"
# pltTitle = "Paris, France"


#===============================================================================
# urlShpFile = homeDir+"/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.shp"
# graphPicklePath = homeDir+"/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.gpickle"
# pltTitle = "OSU SmithLab, Floor 2"
#===============================================================================

# urlShpFile = homeDir + "/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.shp"
# graphPicklePath = homeDir+ "/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.gpickle"


_alphabetSize = 72
_matchThreshold = 0.05
_edgeMatchThreshold = 0.001

dataURL = homeDir + "/Data/TrajectorySearch/"

def matchRoadEdge(graphPicklePath):
    roadGraph = nx.read_gpickle(graphPicklePath)
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
    _path = nx.dijkstra_path(roadGraph, _node1, _node2, None)
    
    pathGraph = nx.subgraph(roadGraph, _path)
    termnodelist = [_node1,_node2]
    
    edgeList = nx.to_edgelist(roadGraph, nodelist=None)
    pathEdgeList = nx.to_edgelist(pathGraph, nodelist = None)
    
    contourAttribute = nx.get_edge_attributes(roadGraph, 'contour')
    pathContourAttribute = nx.get_edge_attributes(pathGraph, 'contour')
    
    # ---------------------------------------------------------------------------
    for i, edge in enumerate(pathEdgeList):
#         print i, (edge[0],edge[1]), pathContourAttribute[(edge[0],edge[1])]
        pass
        
    print termnodelist
    # ---------------------------------------------------------------------------
    
    for i, pedge in enumerate(pathEdgeList):
        pathEdgeFeat = pathContourAttribute[(pedge[0],pedge[1])]
        _matchScores = []
        for j, gedge in enumerate(edgeList):
            graphEdgeFeat = contourAttribute[(gedge[0], gedge[1])]
            
            seq = difflib.SequenceMatcher(None, pathEdgeFeat, graphEdgeFeat)
            _match = seq.ratio()
            _matchScores.append(_match)
#             print i, j, _match
            pass
        
        hist = np.histogram(_matchScores, bins=10)
        print i, hist[0]
        
        pass
    pass

def getCandidateEdgeList(roadGraph, pathList, iPath):
    edgeList = []
    path = pathList[iPath]
    node = path[-1]
    
    
    _edgeList = nx.edges(roadGraph, node)
    for edge in _edgeList:
        if edge[1] in path:
            pass
        else:
            edgeList.append(edge)

    return edgeList
    
    
    pass

def getContourScore(contourAttribute, edge1, edge2):
    contourFeat = 0
    try:
        contourFeat = contourAttribute[(edge1,edge2)]
    except:
        try:
            contourFeat = contourAttribute[(edge2,edge1)]
        except:
            print '.',
        pass
    return contourFeat
    
    pass

def matchPaths(graphPicklePath):
    pathList = []
    edgeList = []
    pathScore = []
    
    
    roadGraph = nx.read_gpickle(graphPicklePath)
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
    _query = nx.astar_path(roadGraph, _node1, _node2, None)
    
    queryGraph = nx.subgraph(roadGraph, _query)
    termnodelist = [_node1,_node2]
    
    edgeList = nx.to_edgelist(roadGraph, nodelist=None)
    edgeLst = roadGraph.edges(data=False)
    queryEdgeList = nx.to_edgelist(queryGraph, nodelist = None)
    
    nQueryEdges = len(queryEdgeList)
    
    contourAttribute = nx.get_edge_attributes(roadGraph, 'contour')
    qContourAttribute = nx.get_edge_attributes(queryGraph, 'contour')
    
    maxEdges = len(edgeList)                # total number of edges in the graph
    # the pathList initially is single edge and contains all edges in the graph

    pathList = []
    for edge in edgeList:
        _edge = [edge[0], edge[1]]
        pathList.append(_edge)
        _edge = [edge[1], edge[0]]
        pathList.append(_edge)
        pass
     
#     pathList = edgeList
    pathScoreList = []
    
    qEdge = queryEdgeList[0]
    qEdgeFeat = qContourAttribute[(qEdge[0],qEdge[1])]
    
#     print pathList
    
    for iPath, pEdge in enumerate(pathList):
        pEdgeFeat = getContourScore(contourAttribute, pEdge[0], pEdge[1])
        
        seq = difflib.SequenceMatcher(None, qEdgeFeat, pEdgeFeat)
        pEdgeScore = seq.ratio()
        pathScoreList.append(pEdgeScore)
        pass
    
    # pathList and corresponding match scorelist are now initialized
#     print pathList[:10]
#     print pathScoreList[:10]
    
    for iQuery, qEdge in enumerate(queryEdgeList):
        plt.figure(1, figsize=(12,12))
        if iQuery > 0:
            _pathList = []
            _pathScoreList = []
            
            
            qEdgeFeat = qContourAttribute[(qEdge[0],qEdge[1])]
            # use updated edgeList, this contains all viable candidate edges in the paths
            
            for iPath, pEdges in enumerate(pathList):
                _edgeList = getCandidateEdgeList(roadGraph, pathList, iPath)
                
                for iEdge, _edge in enumerate(_edgeList):
                    _edgeFeat = getContourScore(contourAttribute, _edge[0], _edge[1])
                    seq = difflib.SequenceMatcher(None, qEdgeFeat, _edgeFeat)
                    _edgeScore = seq.ratio()
                       
                    _pathScore = ( iQuery * pathScoreList[iPath] + _edgeScore ) / (iQuery+1)
                    
                    if _pathScore > _matchThreshold or _edgeScore > _edgeMatchThreshold:
                        
                        _path = pEdges
                        _path.append(_edge[-1])
                        _pathList.append(_path)
                        _pathScoreList.append(_pathScore)
                        pass
                    
                    pass
                pass
            
            pathListSorted = [x for (y,x) in sorted(zip(_pathScoreList,_pathList), reverse=True)]
            pathScoreListSorted = [y for (y,x) in sorted(zip(_pathScoreList,_pathList), reverse=True)]
            
            print iQuery, len(pathList), len(pathListSorted), pathScoreListSorted[0]
            nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='b')
            nx.draw_networkx(queryGraph, pos=shpLayout, edgelist=None, node_size=40, node_color='r', node_shape='o', edgewidth=8, edge_color='r', with_labels=False)
            for __path in pathList[0]:
                subG = nx.subgraph(roadGraph, __path)
                nx.draw_networkx(subG, pos=shpLayout, edgelist=None, node_size=40, node_color='k', node_shape='o', edgewidth=8, edge_color='k', with_labels=False)
            plt.show()
            
            if len(_pathList) == 0:
                break;
                
                
                pass
            
            pathList = pathListSorted
            pathScoreList = pathScoreListSorted
            _pathList = []
            _pathScoreList = []
            
#             for idx in xrange(len(pathList)):
#                 print iQuery, pathScoreList[idx], len(pathList[idx]), len(pathList[idx]), nQueryEdges
            
        pass
    
    print pathList
    pass

def findPath(graphPicklePath):
    pathList = []
    edgeList = []
    pathScore = []
    
    
    roadGraph = nx.read_gpickle(graphPicklePath)
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
    _query = nx.astar_path(roadGraph, _node1, _node2, None)
    
    __query = zip(_query[0:-1], _query[1:])
    
    queryGraph = nx.subgraph(roadGraph, _query)
    termnodelist = [_node1,_node2]
    
    edgeList = nx.to_edgelist(roadGraph, nodelist=None)
    edgeLst = roadGraph.edges(data=False)
    queryEdgeList = nx.to_edgelist(queryGraph, nodelist = None)
    
    nQueryEdges = len(queryEdgeList)
    
    contourAttribute = nx.get_edge_attributes(roadGraph, 'contour')
    qContourAttribute = nx.get_edge_attributes(queryGraph, 'contour')
    
    maxEdges = len(edgeList)                # total number of edges in the graph
    # the pathList initially is single edge and contains all edges in the graph
    
    qEdge = __query[0]
    qEdgeFeat = getContourScore(contourAttribute, qEdge[0], qEdge[1])

    pathList = []
    for edge in edgeList:
        _edge = [edge[0], edge[1]]
        pathList.append(_edge)
        _edge = [edge[1], edge[0]]
        pathList.append(_edge)
        pass
    
    
    # ------------------------------- Force add
    pathList.append([qEdge[0], qEdge[1]])
    # ---------------------------------------
     
#     pathList = edgeList
    pathScoreList = []
    
    
    # ---- plot paths
    exptCount = 0
    
    FigurePath = dataURL + 'Expt' + str(exptCount) + '/'
    while os.path.exists(FigurePath):
        exptCount += 1
        FigurePath = dataURL + 'Expt' + str(exptCount) + '/'
        pass
    os.mkdir(FigurePath)
    
    
    for iPath, pEdge in enumerate(pathList):
        pEdgeFeat = getContourScore(contourAttribute, pEdge[0], pEdge[1])
        
        seq = difflib.SequenceMatcher(None, qEdgeFeat, pEdgeFeat)
        pEdgeScore = seq.ratio()
        pathScoreList.append(pEdgeScore)
        pass
    
    
    cmap = matplotlib.cm.get_cmap('jet')
    
    for iQuery, qEdge in enumerate(__query):
        fig = plt.figure(1, figsize=(20,30))
        figno = "%03d"%iQuery
        figurePath = FigurePath + 'fig' + str(figno) + '.png'
        if iQuery > 0:
            _pathList = []
            _pathScoreList = []
            _edgeScoreList = []
            # ------------------------- Append query to _pathlist ----------
            _pathList.append(_query[0:iQuery+2])
            _pathScoreList.append(1)
            # -------------------------------------------------------------
            
            qEdgeFeat = getContourScore(contourAttribute,qEdge[0],qEdge[1])
            # use updated edgeList, this contains all viable candidate edges in the paths
            
            for iPath, pEdges in enumerate(pathList):
                _edgeList = getCandidateEdgeList(roadGraph, pathList, iPath)
                
                for iEdge, _edge in enumerate(_edgeList):
                    _edgeFeat = getContourScore(contourAttribute, _edge[0], _edge[1])
                    seq = difflib.SequenceMatcher(None, qEdgeFeat, _edgeFeat)
                    _edgeScore = seq.ratio()
                    _edgeScoreList.append(_edgeScore)   
                    _pathScore = ( iQuery * pathScoreList[iPath] + _edgeScore ) / (iQuery+1)
                    
                    if _pathScore > _matchThreshold or _edgeScore > _edgeMatchThreshold:
                        
                        _path = pEdges
                        _path.append(_edge[-1])
                        _pathList.append(_path)
                        _pathScoreList.append(_pathScore)
                        pass
                    pass
                
                pass
            
            pathListSorted = [x for (y,x) in sorted(zip(_pathScoreList,_pathList), reverse=True)]
            pathScoreListSorted = [y for (y,x) in sorted(zip(_pathScoreList,_pathList), reverse=True)]
            
            print iQuery, len(pathList), len(pathListSorted), pathScoreListSorted[0]
            cm = plt.get_cmap('jet')
            ax1 = plt.subplot2grid((3,2),(1, 0),colspan=2,rowspan=2)
#             plt.title(pltTitle)
            nx.draw_networkx_edges(roadGraph, pos=shpLayout, edgelist=None, width=1, edge_color='k', alpha=0.2, edge_cmap=cm)
            black_line = mlines.Line2D([], [], color='k', marker='', markersize=0, label='Road Network')
            blue_line = mlines.Line2D([], [], color='b', marker='', markersize=0, label='Candidate Path')
            plt.legend(handles=[blue_line, black_line])
            for _ipath,__path in enumerate(pathListSorted):
                subG = nx.subgraph(roadGraph, __path)
#                 c = cm(pathScoreListSorted[_ipath])
                for u,v,d in subG.edges(data=True):
                    d['weight'] = pathScoreListSorted[_ipath]

                edges,weights = zip(*nx.get_edge_attributes(subG,'weight').items())
#                 nx.draw(subG, pos=shpLayout, edgelist=zip(__path[0:-1], __path[1:]), node_size=1, node_color='k', node_shape='d', width=1.5, edge_color=weights, with_labels=False, edge_cmap=cm)
                nx.draw_networkx_edges(subG, pos=shpLayout, edgelist=edges, node_size=1, node_color=cm(pathScoreListSorted[_ipath]), node_shape='d', width=1.5, edge_color=clrs.rgb2hex(clrs.colorConverter.to_rgb(cmap(pathScoreListSorted[_ipath]))), with_labels=False)
                pass
            
            ax2 = plt.subplot2grid((3,2), (0,0))
            plt.title("Trajectory")
            nx.draw_networkx_edges(queryGraph, pos=shpLayout, edgelist=__query[:iQuery], node_size=0, node_color='k', style='solid', node_shape='.', width=3, edge_color='k', with_labels=False)
#             nx.draw_networkx_edges(queryGraph, pos=shpLayout, edgelist=__query[:iQuery], node_size=0, node_color='w', style='dashed', node_shape='.', width=1, edge_color='w', with_labels=False)
            plt.xticks([])
            plt.xlabel("")
            plt.yticks([])
            plt.ylabel("")
            
            ax3 = plt.subplot2grid((3,2), (0,1))
            plt.title("Candidate Path Likelihood")
            try:
                n, bins, patches = plt.hist(pathScoreListSorted, bins=100, alpha=0.5, normed=False, color='r', log=True)
                bin_centers = 0.5 * (bins[:-1] + bins[1:])
                # scale values to interval [0,1]
                col = bin_centers - min(bin_centers)
                col /= max(col)
                for c, p in zip(col, patches):
                    plt.setp(p, 'facecolor', cm(c))
            except:
                pass
            plt.ylabel("Number of Paths")
            plt.xlabel("Likelihood")
            
            fig.tight_layout()
            plt.savefig(figurePath)
            
            pathList = pathListSorted
            pathScoreList = pathScoreListSorted
            _pathList = []
            _pathScoreList = []
             
        pass
    
    print pathList
    pass

if __name__ == '__main__':
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    maploc = options.input
    for maploc in ['columbus', 'montpellier', 'washington', 'karlsruhe', 'indoor']:
        if maploc=='montpellier':
            urlShpFile = homeDir+"/Data/FixedShapeFiles/montpellierosmroads/montpellier.osm-roads.shp"
            graphPicklePath = homeDir+"/Data/FixedShapeFiles/montpellierosmroads/montpellier.osm-roads.gpickle"
            pass
        elif maploc=='washington':
            urlShpFile = homeDir+"/Data/fixedRoads/tl_2014_11001_roads.shp"
            graphPicklePath = homeDir+"/Data/fixedRoads/tl_2014_11001.gpickle"
            pass
        elif maploc=='karlsruhe':
            urlShpFile = '/Users/gupta.637/Data/FixedShapeFiles/karlsruhe_germany_osm_roads/karlsruhe_germany_osm_roads.shp'
            graphPicklePath = '/Users/gupta.637/Data/FixedShapeFiles/karlsruhe_germany_osm_roads/karlsruhe_germany_osm_roads.gpickle'
            pass
        elif maploc=='columbus':
            urlShpFile = homeDir+"/Data/fixedRoads/tl_2014_39049_roads.shp"
            graphPicklePath = homeDir+"/Data/fixedRoads/tl_2014_39049.gpickle"
            pass
        elif maploc=='indoor':
            urlShpFile = homeDir+"/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.shp"
            graphPicklePath = homeDir+"/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.gpickle"
        else:
            print 'invalid input, using Columbus map as default...'
            graphPicklePath = homeDir+"/Data/fixedRoads/tl_2014_39049.gpickle"
            pass
        findPath(graphPicklePath)
    
    pass