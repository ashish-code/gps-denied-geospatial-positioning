from itertools import izip
import time
import numpy as np
import scipy.stats as stats
from copy import copy,deepcopy
import networkx as nx
import random
import sys
import heapq
import fnmatch
import re

import matplotlib.colors as colors
import matplotlib.cm as cm
import matplotlib.transforms as transforms
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import LineCollection,EllipseCollection,CircleCollection,PathCollection
from Statistics import *
from Utils import inv_chol,fit_circle_with_endpoints,line_intersection_2d,SpatialAngleNMS

#BIDI_OFFSET = 0.0015 # amount of rendered space between two-way roads
#END_OFFSET = 0.002
#ARROW_SIZE = 0.005
#ROAD_LINE_WIDTH = 0.0005
#TEXT_OFFSET = 0
#TEXT_SIZE = 0.005
#BIDI_OFFSET = 0.0015
#BIDI_OFFSET = 0.005
#DIST_LINE_WIDTH = 0.02
DIST_LINE_WIDTH = 0.03
END_OFFSET = 0
GTCIRC_SIZE = 0.005
#GTCIRC_LINE_WIDTH = 0.0015
#GTPATH_LINE_WIDTH = 0.005
GTCIRC_LINE_WIDTH = 0.004
GTPATH_LINE_WIDTH = 0.015

SIMPLIFY_KLTHRESH = 0.001

# A mode is kept if streetMarginThresh percent of its probability mass is inside [-streetMargin,len+streetMargin]
streetMargin = 0
streetMarginThresh = 0.001
#streetMargin = 15.0/1000.0
##streetMargin = 5.0/1000.0
#streetMarginThresh = 0.5
#streetMargin = 30.0/1000.0
#streetMarginThresh = 0.99
#streetMargin = 50.0/1000.0
#streetMarginThresh = 0.5
#streetMargin = 50.0/1000.0
#streetMarginThresh = -1

SINK_ROAD = 'SINK_ROAD_KEY'

def get_map_display_params(debugDisplay):
    if debugDisplay: 
        ARROW_SIZE = 0.0025
        BIDI_OFFSET = 0.0015
        ROAD_LINE_WIDTH = 0.00025
        TEXT_SIZE = 0.001
        TEXT_OFFSET = 0.001
    else:
        ARROW_SIZE = 0
        BIDI_OFFSET = 0
#        ROAD_LINE_WIDTH = 0.0015
        ROAD_LINE_WIDTH = 0.005
        TEXT_SIZE = 0.01
        TEXT_OFFSET = 0.005
    return (ARROW_SIZE, BIDI_OFFSET, ROAD_LINE_WIDTH, TEXT_SIZE, TEXT_OFFSET)            

def is_intersection(xlim1,ylim1,xlim2,ylim2):
    xBnd = (xlim1[0] > xlim2[0] and xlim1[0] < xlim2[1]) or (xlim1[1] > xlim2[0] and xlim1[1] < xlim2[1]) or \
           (xlim2[0] > xlim1[0] and xlim2[0] < xlim1[1]) or (xlim2[1] > xlim1[0] and xlim2[1] < xlim1[1])
    yBnd = (ylim1[0] > ylim2[0] and ylim1[0] < xlim2[1]) or (ylim1[1] > ylim2[0] and ylim1[1] < ylim2[1]) or \
           (ylim2[0] > ylim1[0] and ylim2[0] < xlim1[1]) or (ylim2[1] > ylim1[0] and ylim2[1] < ylim1[1])
    return xBnd and yBnd

def is_inbounds(pt,ptMin,ptMax):
    return np.all(ptMin.reshape(pt.shape) <= pt) and np.all(pt <= ptMax.reshape(pt.shape))

def get_circarc(r,C,startTheta,endTheta,bidiOff = 0,endOff = 0,arrowOff = 0):
    if bidiOff:
        r = r + np.sign(endTheta - startTheta)*bidiOff
    length = np.abs(startTheta - endTheta)*np.abs(r)
    if length > (2*endOff + arrowOff):
        if startTheta < endTheta:
            arcPath = Path.arc( 180.0*(startTheta + endOff/r)/np.pi,   180.0*(endTheta - endOff/r - arrowOff/r)/np.pi )
        else:
            arcPath = Path.arc(   180.0*(endTheta + endOff/r + arrowOff/r)/np.pi, 180.0*(startTheta - endOff/r)/np.pi )
    else:
        if startTheta < endTheta:
            arcPath = Path.arc( 180.0*startTheta/np.pi,   180.0*endTheta/np.pi )
        else:
            arcPath = Path.arc(   180.0*endTheta/np.pi, 180.0*startTheta/np.pi )

    transM = np.array(((r,0,C[0]),(0,r,C[1]),(0,0,1)))
    transArcPath = arcPath.transformed(transforms.Affine2D(transM))
    return transArcPath

class MapGraph(nx.DiGraph):
    totalLength = None
    posMin = None
    posMax = None
    dispLines = None
    dispPolys = None
    dispPaths = None
    dispDebug = None

    def matchNode(self,nodeStr):
        reExpr = re.compile(fnmatch.translate(nodeStr))
        retNodes = []
        for i in self.nodes_iter():
            #if i.startswith(nodeStr):
            if reExpr.match(i):
                retNodes.append(i)
        return retNodes
                    

    def __init__(self,roads,intersections,useLargestSubgraph = True):
        #leapfrogLen = 5.0/1000.0 # Add leapfrog edges for segments under 5m
        #leapfrogLen = 15.0/1000.0 # Add leapfrog edges for segments under 15m
        leapfrogLen = 30.0/1000.0 # Add leapfrog edges for segments under 30m
        #leapfrogLen = 50.0/1000.0 # Add leapfrog edges for segments under 50m
        #leapfrogLen = 200.0/1000.0 # Add leapfrog edges for segments under 50m

        super(MapGraph, self).__init__()

        self.dispLines = None
        self.dispPolys = None
        self.dispPaths = None
        self.dispDebug = None
        self.posAvg = 0
        self.posMin = np.inf
        self.posMax = -np.inf
        self.totalLength = 0
        minLen = np.inf


        compute_derived_quantities(roads,intersections)
        intEdges = compute_intersecting_roads(roads,intersections,False)
        for (cnode,croadData) in roads.iteritems():
            cstartInt = croadData['start_int']
            cendInt = croadData['end_int']
            cOneWay = croadData['oneway']
            startInt = intersections[cstartInt]
            endInt = intersections[cendInt]
            startPos = startInt['position']
            endPos = endInt['position']

            assert(croadData['type'] == 'line' or croadData['type'] == 'arc')
            assert croadData['length'] > 0

            assert cOneWay != 0
            cnodeKey = cnode
            if cOneWay >= 0:
#                cnodeKey = (cnode,1)
                
                self.add_node(cnodeKey,deepcopy(croadData))
                self.node[cnodeKey]['origin'] = startPos
                self.node[cnodeKey]['terminus'] = endPos
                if 'nameDisplayHint' not in self.node[cnodeKey]:
                    #self.node[cnodeKey]['nameDisplayHint'] = False
                    self.node[cnodeKey]['nameDisplayHint'] = 'name' in self.node[cnodeKey] 

                self.totalLength += self.node[cnodeKey]['length']
                self.posAvg += self.node[cnodeKey]['length']*0.5*(startPos + endPos)

            if cOneWay <= 0:
#                cnodeKey = (cnode,-1)

                self.add_node(cnodeKey,deepcopy(croadData))
                self.node[cnodeKey]['origin'] = endPos
                self.node[cnodeKey]['terminus'] = startPos
                if croadData['type'] == 'line':
                    self.node[cnodeKey]['direction'] = -croadData['direction']
                    self.node[cnodeKey]['start_direction'] = self.node[cnodeKey]['direction']
                    self.node[cnodeKey]['end_direction'] = self.node[cnodeKey]['direction']
                else:
                    self.node[cnodeKey]['start_direction'] = -croadData['end_direction']
                    self.node[cnodeKey]['end_direction'] = -croadData['start_direction']
                    self.node[cnodeKey]['direction'] = self.node[cnodeKey]['start_direction']
                    self.node[cnodeKey]['start_theta'] = copy(croadData['end_theta'])
                    self.node[cnodeKey]['end_theta'] = copy(croadData['start_theta'])
                    self.node[cnodeKey]['curve_alpha'] = -croadData['curve_alpha'] 
                    self.node[cnodeKey]['turn_direction'] = -croadData['turn_direction']
                self.node[cnodeKey]['curve_beta'] = np.arctan2(self.node[cnodeKey]['start_direction'][1],self.node[cnodeKey]['start_direction'][0])
                        
                if 'nameDisplayHint' not in self.node[cnodeKey]:
                    #self.node[cnodeKey]['nameDisplayHint'] = False
                    self.node[cnodeKey]['nameDisplayHint'] = 'name' in self.node[cnodeKey] 

                self.totalLength += self.node[cnodeKey]['length']
                self.posAvg += self.node[cnodeKey]['length']*0.5*(startPos + endPos)

        self.posAvg /= self.totalLength
        
        for (cint,cintData) in intersections.iteritems():
            cint_position = cintData['position']
            origin_roads = intEdges[cint]['origin_roads']
            terminal_roads = intEdges[cint]['terminal_roads']

            self.posMin = np.minimum(self.posMin,cint_position)
            self.posMax = np.maximum(self.posMax,cint_position)
            
            for troad in terminal_roads:
                tdir = self.node[troad]['end_direction']
                totheta = self.node[troad]['curve_alpha']*self.node[troad]['length']
                ttheta = np.arctan2(tdir[1],tdir[0])
                trans_dist = self.node[troad]['length']
                for oroad in origin_roads:
                    if self.node[oroad]['type'] == 'line':
                        odir = self.node[oroad]['direction']
                    else:  
                        odir = self.node[oroad]['start_direction']
                    otheta = np.arctan2(odir[1],odir[0])
                    cangle = otheta - ttheta
                    if cangle > np.pi:
                        cangle += -2.0*np.pi
                    elif cangle < -np.pi:
                        cangle +=  2.0*np.pi
                    cangle = cangle + totheta
                    trans_angle = self.node[troad]['curve_alpha']*self.node[troad]['length']+self.node[troad]['curve_beta']-self.node[oroad]['curve_beta']
                    if trans_angle > np.pi:
                        trans_angle += -2.0*np.pi
                    elif trans_angle < -np.pi:
                        trans_angle +=  2.0*np.pi
                    if np.abs(np.abs(trans_angle) - np.pi) > np.pi/64.0:
                        self.add_edge(troad,oroad,copy(cintData))
                        self[troad][oroad].update({'angle':cangle,\
                                                   'transition_angle':trans_angle,\
                                                   'transition_absangle':np.abs(trans_angle),\
                                                   'transition_distance':trans_dist,\
                                                   'transition_lb':trans_dist,\
                                                   'transition_ub':trans_dist+self.node[oroad]['length'], \
                                                   'skip_set':set()})

        if useLargestSubgraph:
            wcc = nx.weakly_connected_components(self)
            for (n,rmnodes) in enumerate(wcc):
                if n == 0:
                    continue
                self.remove_nodes_from(rmnodes)
            print 'Using largest connected subgraph with {0} roads'.format(len(self))


        # Add a sink node
        self.add_node(SINK_ROAD,{'name':SINK_ROAD,'length':np.inf,'curve_alpha':0,'curve_beta':0})
        
        for cnode in self.nodes_iter():
            if cnode == SINK_ROAD:
                continue
            
            base_od = self.out_degree(cnode)

            # If no outbound edges, add connection to sink.
            if base_od == 0:
                self.add_edge(cnode,SINK_ROAD,{'angle':0,'transition_angle':0,'transition_absangle':0,\
                                               'transition_distance':self.node[cnode]['length'], \
                                               'transition_lb':self.node[cnode]['length'],'transition_ub':np.inf, \
                                               'skip_set':set()})
                base_od = 1

            # Compute path probabilty for directly connected road segments
            for nnode in self.successors(cnode):
                self[cnode][nnode]['base_out_degree'] = base_od
                self[cnode][nnode]['path_prob'] = 1.0/base_od
                
            # Add links to remain on the same road
            self.add_edge(cnode,cnode,{'angle':0,'transition_angle':0,'transition_absangle':0,'transition_distance':0,\
                                       'transition_lb':-np.inf,'transition_ub':self.node[cnode]['length'], \
                                       'path_prob':1, 'skip_set':set()})

        # add leapfrog edges
        leapedEdges = set()
        newEdges = []
        checkEdges = self.edges_iter(data=True)
        while True:
            for (cnode,vt,sEdge) in checkEdges:
                if cnode == vt:
                  continue
                if sEdge['transition_distance'] < leapfrogLen and (cnode,vt) not in leapedEdges:
                    leapedEdges.add((cnode,vt))
                    #print 'Leapfrogging {0}->{1}'.format(cnode,vt)
                    for vt_1 in self.predecessors_iter(cnode):
                        pEdge = self[vt_1][cnode]
                        if vt_1 == cnode or vt_1 == vt or self.has_edge(vt_1,vt):
                            continue
                        if vt_1 in sEdge['skip_set'] or vt in pEdge['skip_set']:
                            continue
                        trans_angle = self.node[vt_1]['curve_alpha']*self.node[vt_1]['length']+self.node[vt_1]['curve_beta']-self.node[vt]['curve_beta']
                        if trans_angle > np.pi:
                            trans_angle += -2.0*np.pi
                        elif trans_angle < -np.pi:
                            trans_angle +=  2.0*np.pi
                        trans_absangle = pEdge['transition_absangle']+np.abs(sEdge['angle'])
                        if trans_absangle > np.pi:
                            continue
                        if len(pEdge['skip_set']&sEdge['skip_set']) > 0:
                            continue
                        skip_set = pEdge['skip_set']|sEdge['skip_set']
                        skip_set.add(cnode)
                        nEdge = { 'angle':pEdge['angle']+sEdge['angle'],\
                                  'path_prob':pEdge['path_prob']*sEdge['path_prob'],\
                                  'transition_angle':trans_angle,\
                                  'transition_absangle':trans_absangle,\
                                  'transition_distance':pEdge['transition_distance']+sEdge['transition_distance'],\
                                  'transition_lb':sEdge['transition_lb']+pEdge['transition_lb'],\
                                  'transition_ub':sEdge['transition_lb']+pEdge['transition_lb']+self.node[vt]['length'],\
                                  'skip_set':skip_set }
                        newEdges.append((vt_1,vt,nEdge))
            if len(newEdges) > 0:
                self.add_edges_from(newEdges)
                checkEdges = newEdges
                newEdges = []
            else:
                break

        self.computeDerivedQuantities()

        print 'Constructed mapgraph with {0} roads'.format(len(self))
        print '\ttotalLength = {0}'.format(self.totalLength)

        # Remove node and edge attributes which are either deprecated or only needed to setup the graph
        for (v,nData) in self.nodes_iter(data = True):
            if v == SINK_ROAD:
                continue
            del nData['start_int']
            del nData['end_int']
            del nData['oneway']
            
        for (vi,vj,eData) in self.edges_iter(data = True):
            del eData['skip_set']
            del eData['angle']
            del eData['transition_absangle']
            if 'name' in eData:
                del eData['name']


    def crop(self,cropMin,cropMax):
        for (vt,nodeData) in self.nodes_iter(data = True):
            cminPos = nodeData['min_pos']
            cmaxPos = nodeData['max_pos']
            if is_intersection((cminPos[0],cmaxPos[0]),(cminPos[1],cmaxPos[1]),(cropMin[0],cropMax[0]),(cropMin[1],cropMax[1])):
                # Keep this road
                continue

            # Creating new edges to the sink to disconnect vt
            for vt_1 in self.predecessors_iter():
                if vt == vt_1:
                    continue
                if self.has_edge(vt_1,SINK_ROAD):
                    self[vt_1][SINK_ROAD]['path_prob'] += self[vt_1][vt]['path_prob']
                    self[vt_1][SINK_ROAD]['transition_lb'] = np.minimum(self[vt_1][SINK_ROAD]['transition_lb'],self[vt_1][vt]['transition_lb'])
                else:
                    self.add_edge(vt_1,SINK_ROAD,self[vt_1][vt])
                    self[vt_1][SINK_ROAD]['transition_ub'] = np.inf

            # Remove the node and it's associated edge data
            self.remove_node(vt)

        self.computeDerivedQuantities()


    def computeDerivedQuantities(self):
        self.posAvg = 0
        self.posMin = np.inf
        self.posMax = -np.inf
        self.totalLength = 0
        for (vt_1,nodeData) in self.nodes_iter(data = True):
            if vt_1 == SINK_ROAD:
                continue
                
            startPos = nodeData['origin']
            endPos = nodeData['terminus']
            cLen = nodeData['length']

            # compute summary statistics
            if nodeData['type'] == 'line':
                nodeData['min_pos'] = np.minimum(startPos,endPos)
                nodeData['max_pos'] = np.maximum(startPos,endPos)
                self.posAvg += cLen*0.5*(startPos + endPos)
            elif nodeData['type'] == 'arc':
                p = self.get_road_position(vt_1,np.linspace(0,cLen,5))
                nodeData['min_pos'] = np.min(p,1)
                nodeData['max_pos'] = np.max(p,1)
                self.posAvg += cLen*np.mean(p,1)
            else:
                assert False

            self.totalLength += cLen
            self.posMin = np.minimum(self.posMin,nodeData['min_pos'])
            self.posMax = np.maximum(self.posMax,nodeData['max_pos'])

            # compute transition information for each node
            outdeg_vt_1 = self.out_degree(vt_1)
            cdfPos = np.empty((outdeg_vt_1,2))
            pathProb = np.empty((outdeg_vt_1,1))
            nodeInd = dict()
            for (i,vt) in enumerate(self.successors_iter(vt_1)):
                cdfPos[i,0] = self[vt_1][vt]['transition_lb']
                cdfPos[i,1] = self[vt_1][vt]['transition_ub']
                pathProb[i,0] = self[vt_1][vt]['path_prob']
                nodeInd[vt] = i
            self.node[vt_1]['transition_distances'] = cdfPos
            self.node[vt_1]['transition_path_prob'] = pathProb
            self.node[vt_1]['transition_node_index'] = nodeInd
        self.posAvg /= self.totalLength
    
    def printNode(self,roadKey):
        printFields = ['segment_type','type','length','curve_alpha','curve_beta','transition_node_index','transition_distances','transition_path_prob']
        print '\nNode {0}:'.format(roadKey)
        for name in printFields:
            val = self.node[roadKey][name]
            print '  {0}: {1}'.format(name,repr(val))
        print '  pred = ',
        for i in self.predecessors_iter(roadKey):
            if i != roadKey:
                print '{0} '.format(i),
        print
        print '  succ = ',
        for i in self.successors_iter(roadKey):
            if i != roadKey:
                print '{0} '.format(i),
        print

    def printEdge(self,v0,v1):
        print '\nEdge {0} -> {1}:'.format(v0,v1)
        for (name,val) in self[v0][v1].iteritems():
            print '  {0}: {1}'.format(name,repr(val))
        palpha = self.node[v0]['curve_alpha']
        calpha = self.node[v1]['curve_alpha']
        plen = self.node[v0]['length']
        translen = self[v0][v1]['transition_distance']
        transangle = self[v0][v1]['transition_angle']
        print '  angle off: {0}'.format(transangle-palpha*plen+calpha*translen)
        print '  angle scale: {0}'.format(palpha-calpha)

    def distanceToRoad(self,road,pos,direction = None):
        if direction != None:
            directionNorm = np.sqrt(np.sum(np.power(direction,2.0)))
        else:
            directionNorm = 0
        cnodeData = self.node[road]
        if cnodeData['type'] == 'line':
            rdir = cnodeData['direction']
            d = pos - cnodeData['origin']
            vPerp = copy(rdir)
            vPerp[0] = rdir[1]
            vPerp[1] = -rdir[0]
            alpha = np.dot(rdir.T,d)
            if alpha < 0:
                dist = np.sqrt(np.sum(np.power(d,2.0)))
            elif alpha > cnodeData['length']:
                dist = np.sqrt(np.sum(np.power(pos - cnodeData['terminus'],2.0)))
            else:
                dist = np.abs(np.dot(vPerp.T,d))
            tdir = cnodeData['direction']
        else:
            d = pos - cnodeData['center']
            startTheta = cnodeData['start_theta']
            endTheta = cnodeData['end_theta']
            theta = np.arctan2(d[1],d[0])
            while np.abs(theta - 0.5*(startTheta + endTheta)) > np.pi:
                theta -= np.sign(theta - 0.5*(startTheta + endTheta))*2.0*np.pi
            alpha = cnodeData['radius']*np.sign(endTheta - startTheta)*(theta - startTheta)
            if alpha < 0: 
                dist = np.sqrt(np.sum(np.power(pos - cnodeData['origin'],2.0)))
                rdir = cnodeData['start_direction']
            elif alpha > cnodeData['length']:
                dist = np.sqrt(np.sum(np.power(pos - cnodeData['terminus'],2.0)))
                rdir = cnodeData['end_direction']
            else:
                dist = np.abs(cnodeData['radius'] - np.sqrt(np.sum(np.power(d,2.0))))
                rdir = np.sign(endTheta - startTheta)*np.array([-np.sin(theta),np.cos(theta)])
            tdir = cnodeData['start_direction']

        if directionNorm > 1e-10:
            ttheta = np.arctan2(tdir[1],tdir[0])
            otheta = np.arctan2(direction[1],direction[0])
            dirAngle = otheta - ttheta
            if dirAngle > np.pi:
                dirAngle  = dirAngle - 2.0*np.pi
            elif dirAngle < -np.pi:
                dirAngle  = dirAngle + 2.0*np.pi
            reldirAngle = np.arccos(np.dot(rdir.reshape(1,2),direction.reshape(2,1)/directionNorm))                    
        else:
            dirAngle = 0
            reldirAngle = 0

        return (dist,float(reldirAngle),float(dirAngle),alpha)

    def getNearbyRoads(self,pos,direction,posThresh,dirThresh,alphaThresh,nbhdSet = None):
        roads = dict()
        minDist = np.inf
        minDistData = None
        if nbhdSet == None:
            checkNodes = self.nodes_iter()
        else:
            checkNodes = nbhdSet
        for cnode in checkNodes:
            if cnode == SINK_ROAD:
                continue
            (dist,reldirAngle,dirAngle,alpha) = self.distanceToRoad(cnode,pos,direction)

#            print dist,alpha
            nearby = True
            if nearby and alphaThresh != None:
                nearby = nearby and (-alphaThresh <= alpha and alpha <= (self.node[cnode]['length'] + alphaThresh))
            if nearby and posThresh != None:
                nearby = nearby and dist <= posThresh
            if nearby and dirThresh != None:
                nearby = nearby and (np.abs(reldirAngle) <= dirThresh)
    
            if np.abs(reldirAngle) <= dirThresh and dist < minDist:
                minDist = dist
                minDistData = (cnode,(dist,reldirAngle,dirAngle,alpha))


            if nearby:
                roads[cnode] = (dist,reldirAngle,dirAngle,alpha)

        return (roads,minDistData)
    
    def get_road_heading(self,croad,d):
        return self.node[croad]['curve_beta'] + np.multiply(self.node[croad]['curve_alpha'],d)
    
    def get_road_position(self,croad,d,offset = 0):
        """ Get the position of a point along a road """
        start_pos = self.node[croad]['origin']
        end_pos = self.node[croad]['terminus']
        length = self.node[croad]['length']
        #bidirectional = self.node[croad]['oneway'] == 0
        bidirectional = True
        doOffset = offset>0 and bidirectional
        nD = len(d)
        if self.node[croad]['type'] == 'line':
            alpha = d/length
            basePos = np.dot(start_pos.reshape([2,1]),(1.0-alpha).reshape([1,nD])) + np.dot(end_pos.reshape([2,1]),alpha.reshape([1,nD]))
#            basePos = start_pos + (end_pos - start_pos)*alpha
            if doOffset:
                baseDir = self.node[croad]['direction'].reshape([2,1]) 
        else:
            C = self.node[croad]['center'].reshape(2)
            r = self.node[croad]['radius']
            startTheta = self.node[croad]['start_theta']
            endTheta = self.node[croad]['end_theta']
            curr_theta = startTheta + (endTheta - startTheta)*(d/length)
            basePos = C.reshape([2,1]) + r*np.array([np.cos(curr_theta).reshape(nD),np.sin(curr_theta).reshape(nD)]).reshape([2,nD])
            if doOffset:
                baseDir = self.node[croad]['turn_direction']*np.array([-np.sin(curr_theta).reshape(nD),np.cos(curr_theta).reshape(nD)]).reshape([2,nD])
        if doOffset:
            offDir = np.array([baseDir[1,:],-baseDir[0,:]])
            return basePos + offset*offDir
        else:
            return basePos

    def display(self,fig,ax = None, gtPos = None, gtPosPath = None, odomPosPath = None, debugDisplay = False):
        if ax == None:
            ax = fig.add_subplot(111)

        ax.set_aspect(1)
        
        ARROW_SIZE, BIDI_OFFSET, ROAD_LINE_WIDTH, TEXT_SIZE, TEXT_OFFSET = \
            get_map_display_params(debugDisplay)

        if self.dispLines == None or self.dispDebug != debugDisplay:
            self.dispDebug = debugDisplay
            self.dispLines = []
            self.dispPolys = []
            self.dispPaths = []
            for cnode in self.nodes_iter():
                if cnode == SINK_ROAD:
                    continue
                #bidirectional = self.node[cnode]['oneway'] == 0
                bidirectional = True
                length = self.node[cnode]['length']
                cArrowSize = np.minimum(ARROW_SIZE,0.5*length)
                if self.node[cnode]['type'] == 'line':
                    origin = self.node[cnode]['origin']
                    direction = self.node[cnode]['direction']
                    normDirection = np.array(((direction[1]),(-direction[0])))
                    
                    if bidirectional:
                        directionOffsets = BIDI_OFFSET
                    else:
                        directionOffsets = 0
    
                    startPt = origin + END_OFFSET*direction + directionOffsets*normDirection
                    endPt = origin + (length-END_OFFSET)*direction + directionOffsets*normDirection
                    if cArrowSize > 0:
                        tri1Pt = endPt - cArrowSize*direction + 0.25*ARROW_SIZE*normDirection
                        tri2Pt = endPt - cArrowSize*direction - 0.25*ARROW_SIZE*normDirection
                        triMidPt = 0.5*(tri1Pt + tri2Pt)
                        self.dispLines.append(np.vstack((startPt.T,triMidPt.T,tri1Pt.T,endPt.T,tri2Pt.T,triMidPt.T)))
                    else:
                        self.dispLines.append(np.vstack((startPt.T,endPt.T)))
#                    self.dispLines.append([origin,origin + 0.1*length*direction])
                else:
                    startDir = self.node[cnode]['start_direction']
                    startTheta = self.node[cnode]['start_theta']
                    startPos = self.node[cnode]['origin']
                    endDir = self.node[cnode]['end_direction']
                    endTheta = self.node[cnode]['end_theta']
                    endNDir = np.array((endDir[1],-endDir[0]))
                    endPos = self.node[cnode]['terminus']
                    r = self.node[cnode]['radius']
                    C = self.node[cnode]['center']
                    startPt = startPos
                    endPt = endPos
                    if bidirectional:
                        directionOffsets = BIDI_OFFSET
                    else:
                        directionOffsets = 0
                    
                    transArcPath = get_circarc(r,C,startTheta,endTheta,directionOffsets,END_OFFSET,cArrowSize)
                    self.dispPaths.append(transArcPath)
                    
                    if cArrowSize > 0:
                        triEndPt = endPos - END_OFFSET*endDir + directionOffsets*endNDir
                        tri1Pt = triEndPt - cArrowSize*endDir + 0.25*ARROW_SIZE*endNDir
                        tri2Pt = triEndPt - cArrowSize*endDir - 0.25*ARROW_SIZE*endNDir
                        triMidPt = 0.5*(tri1Pt + tri2Pt)
                        self.dispLines.append(np.vstack((triMidPt.T,tri1Pt.T,triEndPt.T,tri2Pt.T,triMidPt.T)))

        dx0 = ax.viewLim.width
        dx1 = ax.bbox.width
        sc = (dx1/dx0)*(72.0/fig.dpi)
        intCircSize = np.pi*(END_OFFSET*sc)**2
        
        if debugDisplay:
            for (cnode,cnodeData) in self.nodes_iter(True):
                if cnode == SINK_ROAD:
                    continue
    
                if debugDisplay:
                    nameText = str(cnode)[0:10]
                else:
                    if 'name' in cnodeData and cnodeData['name'] != None and cnodeData['nameDisplayHint']:
                        nameText = cnodeData['name']
                    else:
                        continue
                if len(nameText)*TEXT_SIZE > 5*cnodeData['length']:
                    continue
    
    
                midPt = self.get_road_position(cnode,[0.5*cnodeData['length']],TEXT_OFFSET)
                if cnodeData['type'] == 'line':
                    direction = cnodeData['direction']
                    rot = np.arctan2(direction[1],direction[0])
                else:
                    midTheta = 0.5*(cnodeData['start_theta'] + cnodeData['end_theta'])
                    rot = midTheta + np.pi/2.0
    
                while rot > np.pi:
                    rot = rot - 2.0*np.pi
                while rot < -np.pi:
                    rot = rot + 2.0*np.pi
                if rot > np.pi/2.0:
                    rot = rot - np.pi
                if rot < -np.pi/2.0:
                    rot = rot + np.pi
                rot = (180.0/np.pi)*rot
                ax.text(midPt[0],midPt[1],nameText, \
                        ha = 'center', va = 'center', \
                        size = sc*TEXT_SIZE, rotation = rot, \
                        transform = ax.transData)

        if gtPos != None:
            gtCircSize = np.pi*(GTCIRC_SIZE*sc)**2
            nCircs = 4
            ax.add_collection(CircleCollection(sizes = [gtCircSize*(2.0**(2.0*(i+2))) for i in range(nCircs)], \
                                               offsets = [gtPos.reshape(2) for i in range(nCircs)], \
                                               transOffset = ax.transData, \
                                               facecolors = 'none',linewidths=sc*GTCIRC_LINE_WIDTH,linestyle='dashed'))
        if gtPosPath != None:
#            ax.add_patch(PathPatch(Path(gtPosPath),lw = sc*0.0005,zorder = 100))
            ax.add_collection(PathCollection([Path(gtPosPath)],transOffset = ax.transData,linewidths=sc*GTPATH_LINE_WIDTH,edgecolors = 'black',facecolors = 'none',zorder=100,alpha=0.75));
        if odomPosPath != None:
            ax.add_collection(PathCollection([Path(odomPosPath)],transOffset = ax.transData,linewidths=sc*GTPATH_LINE_WIDTH,edgecolors = 'grey',facecolors = 'none',zorder=101));

        ax.add_collection(LineCollection(self.dispLines,transOffset = ax.transData,linewidths=sc*ROAD_LINE_WIDTH,colors = 'blue'))
        ax.add_collection(PathCollection(self.dispPaths,transOffset = ax.transData,linewidths=sc*ROAD_LINE_WIDTH,edgecolors = 'blue',facecolors = 'none'))
        
        return (fig,ax)
    
class MapDynamics(object):
    state_dim = None
    obs_dim = None

    dt = None
    dt_1 = None

    def __init__(self,stateDim,obsDim):
        self.state_dim = stateDim
        self.obs_dim = obsDim

    def street_state_mask(self, mapgraph, st, vt):
        len = mapgraph.node[vt]
        d = st[0,:]
        return np.logical_and(0 <= d, d <= len)

    def street_transition_probs(self, mapgraph, vt_1, st_1, Sigma_st_1 = None,Normalized = True,computeDerivs = False):
        (A_s,b_s,Sigma_s) = self.state_transition_distribution(mapgraph, vt_1, vt_1, st_1=None,returnChol=False)

        if Sigma_st_1 == None:
            Sigma = Sigma_s[0,0]
        else:
            Sigma = Sigma_s[0,0] + np.dot(A_s[0,:],np.dot(Sigma_st_1,A_s[0,:].T)) 

        x = np.dot(A_s[0,:],st_1)
        
        cdfPos = mapgraph.node[vt_1]['transition_distances'].reshape((1,-1,2))
        pathProb = mapgraph.node[vt_1]['transition_path_prob'].reshape((1,-1))
        vtIndices = mapgraph.node[vt_1]['transition_node_index'] 
        cdfVals = stats.norm.cdf(cdfPos,loc=x.reshape((-1,1,1)),scale=np.sqrt(Sigma))
        cdfVals[np.isnan(cdfVals)] = 0
        ps = np.multiply(cdfVals[:,:,1] - cdfVals[:,:,0],pathProb)
        if Normalized:
            unnorm_ps = ps
            sumps = np.sum(ps,1).reshape(-1,1)
            ps = np.divide(ps,sumps)
        if computeDerivs:
            # compute approximate gradient wrt st_1 for thresholding
            dpVals = stats.norm.pdf(cdfPos,loc=x.reshape((-1,1,1)),scale=np.sqrt(Sigma))
            dpVals[np.isnan(dpVals)] = 0
            dps = np.multiply(dpVals[:,:,1] - dpVals[:,:,0],pathProb)
            #if Normalized:
            #    dps = np.multiply(np.divide(sumps - unnorm_ps,np.power(sumps,2.0)),dps)
            dpsdst_1 = np.dot(A_s[0,:].reshape((-1,1)),dps)
            return ps,dpsdst_1,vtIndices
        else:
            return ps,vtIndices

    def street_transition_logprob(self,mapgraph,vts,vt_1,st_1, Sigma_st_1 = None,Normalized = True):
        ps,vtIndices = self.street_transition_probs(mapgraph,vt_1,st_1, Sigma_st_1, Normalized)
        lp = np.copy(ps)
        lp[ps > 0] = np.log(ps[ps > 0])
        lp[ps <= 0] = -np.inf 
#        lp = np.log(ps)
        if vts == None:
            return lp
        else:
            return lp[vtIndices[vts]]

    def synthesize_path(self, mapgraph, speed, path = None, pathLength = None, noiseless = False):
        cs = self.init_state_from_speed(speed)
        
        transProbThres = 0.5
    
        states = list()
        curr_time = 0
        curr_pathLength = 0

        assert False, 'need to update calls to street_transition_alpha'

        if path == None:
            path = []
            curr_street = None
            next_street = SINK_ROAD
            while next_street == SINK_ROAD:
                next_street = random.choice(mapgraph.nodes())
#            for curr_street_ind in range(pathLength):
            while curr_pathLength < pathLength:
                curr_street = next_street
                while next_street == curr_street or next_street == SINK_ROAD:
                    # FIXME: This is wrong with the leapfrog edges included
                    next_street = random.choice(mapgraph.successors(curr_street))
        
                alpha = 1
#                while curr_pathLength < pathLength and alpha >= transProbThres:
                while curr_pathLength < pathLength and alpha >= transProbThres:
                    alpha = self.street_transition_alpha(mapgraph,True,curr_street,cs)
                    if alpha < transProbThres:
                        (A,b,(Sigma,cholSigma)) = self.state_transition_distribution(mapgraph,next_street,curr_street,returnChol=True)
                    else:
                        (A,b,(Sigma,cholSigma)) = self.state_transition_distribution(mapgraph,curr_street,curr_street,returnChol=True)
                    cs = np.dot(A,cs) + b
                    if not noiseless:
                        cs += np.dot(cholSigma,np.random.randn(self.state_dim,1))
                    curr_time = curr_time + self.dt
                    if alpha < transProbThres:
                        states.append({'street':next_street,'state':cs,'time':curr_time})
                    else:
                        states.append({'street':curr_street,'state':cs,'time':curr_time})
                    curr_pathLength += 1
        else:
#            for curr_street_ind in range(len(path)):
            curr_street_ind = 0
            while curr_pathLength < pathLength and curr_street_ind < len(path):
                curr_street = path[curr_street_ind]
        
                if curr_street_ind+1 < len(path):
                    next_street = path[curr_street_ind+1]
                else:
                    next_street = None
        
                alpha = 1
                while curr_pathLength < pathLength and alpha >= transProbThres:
                    alpha = self.street_transition_alpha(mapgraph, True,curr_street,cs)
                    if alpha < transProbThres and next_street != None:
                        (A,b,(Sigma,cholSigma)) = self.state_transition_distribution(mapgraph,next_street,curr_street,returnChol=True)
                    else:
                        (A,b,(Sigma,cholSigma)) = self.state_transition_distribution(mapgraph,curr_street,curr_street,returnChol=True)
                    cs = np.dot(A,cs) + b
                    if not noiseless:
                        cs += np.dot(cholSigma,np.random.randn(self.state_dim,1))
                    curr_time = curr_time + self.dt
                    if alpha < transProbThres and next_street != None:
                        states.append({'street':next_street,'state':cs,'time':curr_time})
                    else:
                        states.append({'street':curr_street,'state':cs,'time':curr_time})
                    curr_pathLength += 1
                curr_street_ind += 1
                
        return states
    
    def synthesize_odometry(self,mapgraph,states):
        t = 0
        obs = []
        tInd = 0
        for curr_state in states:
            (obsA,obsb,(obsSigma,obsCholSigma)) = self.observation_distribution(mapgraph,curr_state['street'],returnChol=True)
            cobs = {'t':t,'tInd':tInd,'dt':self.dt,'obs':np.dot(obsA,curr_state['state']) + obsb + np.dot(obsCholSigma,np.random.randn(2,1))}
            obs.append(cobs)
            t += self.dt
            tInd += 1
        return obs

class MapState2RigidTurnDynamics(MapDynamics):
    # Observation parameters
    type_map = None
    v_sigma = None
    dtheta_sigma = None
    # Motion model parameters
    dv_sigma = None
    ddtheta_sigma = None
    gamma = None

    kinds = None

    def __init__(self,params,kinds = True):
        super(MapState2RigidTurnDynamics, self).__init__(4,2)
        self.kinds = kinds
        self.setParameters(params)

    def fit(self,sequences):
        if self.kinds:
            osmTypeMap = {'motorway':'highway', 'motorway_link':'highway', 'trunk':'highway', 'trunk_link':'highway', 'primary':'highway', 'primary_link':'highway' }
#            linkedOSMTypes = ['motorway', 'trunk', 'primary', 'secondary', 'tertiary']
#            osmTypeMap = dict([(k,'highway') for k in linkedOSMTypes] + \
#                              [(k+'_link','highway') for k in linkedOSMTypes])
#        if self.kinds:
#            linkedOSMTypes = ['motorway', 'trunk', 'primary', 'secondary', 'tertiary']
#            nonLinkedOSMTypes = ['living_street','residential','service','unclassified','track']
#            allOSMTypes = nonLinkedOSMTypes + linkedOSMTypes + [ linkType+'_link' for linkType in linkedOSMTypes]
#            osmTypeMap = dict([(t,t) for t in allOSMTypes])
        else:
            osmTypeMap = {}
        diffOSMTypes = set(osmTypeMap.itervalues())
        diffOSMTypes.add('default')
        obsData = dict([ (type,[]) for type in diffOSMTypes ])
        transData = dict([ (type,{'D':[],'us':[],'vs':[]}) for type in diffOSMTypes ])
        prevState = None
        baseDT = None
        for (mapgraph,obs,states) in sequences:
            for (N,(curr_obs,currState)) in enumerate(izip(obs,states)):
                tInd = curr_obs['tInd']
                t = curr_obs['t']
                curr_data_dt = curr_obs['dt']
                yt = curr_obs['obs']
                if baseDT == None:
                    baseDT = curr_data_dt
                else:
                    # This is due to the estimation of gamma_theta
                    assert np.abs(baseDT-curr_data_dt) < 1e-10,'Cannot handle inequal time steps in fitting RigidTurnDynamics'

                self.setParameters({'dt':curr_data_dt,'dt_1':self.dt})
                if N == 0:
#                if N == 0 or currState['street'] != prevState['street']:
                    prevState = currState
                    continue

                curr_street = currState['street']
                prev_street = prevState['street']
                if mapgraph.node[curr_street]['osm_kind'] in osmTypeMap:
                    streetType = osmTypeMap[mapgraph.node[curr_street]['osm_kind']]
                else:
                    streetType = 'default'

                (obsMu,obsSigma) = self.observation_distribution(mapgraph, curr_street, currState['state'])
                obsData[streetType].append(yt - obsMu)

                (mu,Sigma) = self.state_transition_distribution(mapgraph, curr_street, prev_street, prevState['state'])
                transData[streetType]['D'].append(mu - currState['state'])
                transData[streetType]['us'].append(currState['state'][3])
                transData[streetType]['vs'].append(currState['state'][2])

                prevState = currState
        
        fitParams = {}
        
#        U = np.array(us)
#        V = np.array(vs)
#
#        angleTransitionAlpha = np.sum(np.multiply(U,V))/np.sum(np.power(U,2))
#        print 'Angle Decay Parameter: {0}'.format(angleTransitionAlpha)
#        fitParams['gamma_theta'] = -np.log(angleTransitionAlpha)/self.dt
#
#        angleTransitionVar = np.mean(np.power(V - angleTransitionAlpha*U,2))
#        print 'Angle Noise Variance: {0} (std dev: {1})'.format(angleTransitionVar,np.sqrt(angleTransitionVar)) 
#        fitParams['ddtheta_sigma'] = np.sqrt(angleTransitionVar)
        
#        errMat = np.array(D)
#        errSigma = np.cov(errMat.reshape(errMat.shape[0],errMat.shape[1]), rowvar=0)
#        print 'Distance Error Variance: {0} (std dev: {1})'.format(errSigma[0,0],np.sqrt(errSigma[0,0]))
#        fitParams['dv_sigma'] = np.sqrt(errSigma[0,0])

        fitParams['type_map'] = osmTypeMap
        fitParams['gamma'] = {}
        fitParams['ddtheta_sigma'] = {}
        fitParams['dv_sigma'] = {}
        fitParams['v_sigma'] = {}
        fitParams['dtheta_sigma'] = {}
        for type in diffOSMTypes:
            us = transData[type]['us']
            vs = transData[type]['vs']
            D = transData[type]['D']
            Dobs = obsData[type]

            print '{0} (N = {1})'.format(type,len(Dobs))
            if len(Dobs) == 0:
                continue
            
            U = np.array(us)
            V = np.array(vs)

            angleTransitionAlpha = np.sum(np.multiply(U,V))/np.sum(np.power(U,2))
            print '  Angle Decay: {0}'.format(angleTransitionAlpha)
            fitParams['gamma'][type] = angleTransitionAlpha

            angleTransitionVar = np.mean(np.power(V - angleTransitionAlpha*U,2))
            print '  Angle Var: {0} (std dev: {1})'.format(angleTransitionVar,np.sqrt(angleTransitionVar)) 
            fitParams['ddtheta_sigma'][type] = np.sqrt(angleTransitionVar)
            
            errMat = np.array(D)
            errSigma = np.cov(errMat.reshape(errMat.shape[0],errMat.shape[1]), rowvar=0)
            print '  Dist Var: {0} (std dev: {1})'.format(errSigma[0,0],np.sqrt(errSigma[0,0]))
            fitParams['dv_sigma'][type] = np.sqrt(errSigma[0,0])
       
            errMat = np.array(Dobs)
            errSigma = np.cov(errMat.reshape(errMat.shape[0],errMat.shape[1]), rowvar=0)
            print '  v std dev',np.sqrt(errSigma[0,0])
            print '  theta std dev',np.sqrt(errSigma[1,1])
            fitParams['v_sigma'][type]= np.sqrt(errSigma[0,0])
            fitParams['dtheta_sigma'][type] = np.sqrt(errSigma[1,1])
        
        return fitParams

    def convert_posorient_sequence(self,mapgraph,posorients):
        states = []
        (prevStreet,prevPos,prevOrient) = posorients[0]
        for (currStreet,pos,orient) in posorients[1:]:
            if currStreet == None:
                prevStreet, prevPos, prevOrient = currStreet, pos, orient
                continue

            if mapgraph.node[currStreet]['type'] == 'line':
                baseOrient = 0
                prevBaseOrient = 0
            else:
                currAlpha = mapgraph.node[currStreet]['curve_alpha']
                baseOrient = pos*currAlpha
                prevBaseOrient = prevPos*currAlpha

            states.append(np.array((pos,prevPos,orient-baseOrient,prevOrient-prevBaseOrient)).reshape(self.state_dim,1))
            prevStreet, prevPos, prevOrient = currStreet, pos, orient 
        return states

    def init_state_from_speed(self,speed):
        return np.array([[0],[-speed*self.dt],[0],[0]])

    def get_state_vel_matrix(self,mapgraph,vt):
        alpha = mapgraph.node[vt]['curve_alpha']
        return np.array([[1,0,0,0],[1.0/self.dt,-1.0/self.dt,0,0],[0,0,1,0],[alpha/self.dt,-alpha/self.dt,1.0/self.dt,-1.0/self.dt]])

    def get_canonical_state_transform(self,mapgraph,vt):
        alpha = mapgraph.node[vt]['curve_alpha']
        beta = mapgraph.node[vt]['curve_beta']
        return (np.array([[1,0,0,0],[alpha,0,1,0]]),np.array([[0],[beta]]))

    def setParameters(self,params):
        fields = set(['dt','dt_1','v_sigma', 'dtheta_sigma', 'dv_sigma', 'ddtheta_sigma', 'gamma', 'type_map'])
        for f in params:
            assert f in fields,'Unrecognized parameter name: {0}'.format(f)
            setattr(self,f,params[f])

    def observation_distribution(self,mapgraph,vt,st = None,returnChol=False):
        alpha = mapgraph.node[vt]['curve_alpha']
        A_y = np.array([[1.0/self.dt,-1.0/self.dt,0,0],[alpha/self.dt,-alpha/self.dt,1.0/self.dt,-1.0/self.dt]])
        b_y = np.array([[0],[0]])

        if self.type_map != None:
            if mapgraph.node[vt]['osm_kind'] in self.type_map:
                streetType = self.type_map[mapgraph.node[vt]['osm_kind']]
            else:
                streetType = 'default'
            v_sigma = self.v_sigma[streetType]
            dtheta_sigma = self.dtheta_sigma[streetType]
        else:
            v_sigma = self.v_sigma
            dtheta_sigma = self.dtheta_sigma

        Sigma_y = np.array([(v_sigma**2,0),(0,dtheta_sigma**2)])
        if returnChol:
            cholSigma_y = np.array([(v_sigma,0),(0,dtheta_sigma)])
            Sigma = (Sigma_y,cholSigma_y)
        else:
            Sigma = Sigma_y

        if st == None:
            return (A_y,b_y,Sigma)
        else:
            return (np.dot(A_y,st) + b_y,Sigma)

    def state_transition_distribution(self, mapgraph, vt, vt_1, st_1=None,returnChol=False):
        if self.type_map != None:
            if mapgraph.node[vt]['osm_kind'] in self.type_map:
                streetType = self.type_map[mapgraph.node[vt]['osm_kind']]
            else:
                streetType = 'default'

            gamma = self.gamma[streetType]
            dv_sigma = self.dv_sigma[streetType]
            ddtheta_sigma = self.ddtheta_sigma[streetType]
        else:
            gamma = self.gamma
            dv_sigma = self.dv_sigma
            ddtheta_sigma = self.ddtheta_sigma

        dtr = (self.dt/self.dt_1)
        dt2 = self.dt**2
        epsVar = (0.00001)**2
        #epsVar = (0.000001)**2
        #epsVar = (0.0000001)**2
        #epsVar = 0
        A_s = np.array([[ 1+dtr, -dtr,       0,  0 ], \
                             [     1,    0,       0,  0 ], \
                             [     0,    0,  gamma,  0 ],\
                             [     0,    0,       1,  0 ]])
        Sigma_s = np.array([[ (dv_sigma)**2,      0,                                       0,      0 ], \
                                 [                      0, epsVar,                                       0,      0 ], \
                                 [                      0,      0,  (ddtheta_sigma)**2,      0 ], \
                                 [                      0,      0,                                       0, epsVar ]])
        cholSigma_s = np.power(Sigma_s,0.5)
        
        if vt == vt_1:
            A = A_s
            b = np.zeros((A_s.shape[0], 1))
            if returnChol:
                Sigma = (Sigma_s,cholSigma_s)
            else:
                Sigma = Sigma_s       
        else:
            palpha = mapgraph.node[vt_1]['curve_alpha']
            calpha = mapgraph.node[vt]['curve_alpha']
            plen = mapgraph.node[vt_1]['length']
            translen = mapgraph[vt_1][vt]['transition_distance']
            angle = mapgraph[vt_1][vt]['transition_angle']
            Aref = A_s
            bref = np.array([[-translen], [-translen], [0], [angle-palpha*plen+calpha*translen]])
            Aupd = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,palpha-calpha,0,1]])
            A = np.dot(Aupd,Aref)
            #b = np.dot(Aupd,bref)
            b = bref
            if returnChol:
                Sigma = (Sigma_s,cholSigma_s)
            else:
                Sigma = Sigma_s
        if st_1 == None:
            return (A, b, Sigma)
        else:
            mu = np.dot(A,st_1) + b
            return (mu,Sigma)

    def get_initial_distribution(self,mapgraph,curr_street,initPos,initPosVar,initSpeed,initSpeedVar,cropRange):
        if isinstance(self.gamma,dict):
            if mapgraph.node[curr_street]['osm_kind'] in self.gamma:
                osmKind = mapgraph.node[curr_street]['osm_kind']
            else:
                osmKind = 'default'

            gamma = self.gamma[osmKind]
            dv_sigma = self.dv_sigma[osmKind]
            ddtheta_sigma = self.ddtheta_sigma[osmKind]
        else:
            gamma = self.gamma
            dv_sigma = self.dv_sigma
            ddtheta_sigma = self.ddtheta_sigma

        transAngVar = ddtheta_sigma**2
        #initAngVar = 10**2*transAngVar/(1.0 - gamma**2)
        initAngVar = (np.pi/4)**2

        if np.isinf(initPosVar):
            clen = mapgraph.node[curr_street]['length']
            calpha = mapgraph.node[curr_street]['curve_alpha']
#            nCompsPerKm = 10 # 50 components per kilometer
#            nCompsPerKm = 50 # 50 components per kilometer
            nCompsPerKm = 100 # 100 components per kilometer
#            nCompsPerKm = 200 # 200 components per kilometer
            nPts = int(np.ceil(nCompsPerKm*clen))
            currDist = GaussianMixtureModel(self.state_dim,nPts)
            
            intWidth = clen/nPts
            compPosVar = (intWidth)**2
            compPrevPosVar = compPosVar+self.dt**2*initSpeedVar

            logw = - np.log(nPts)
            sigma = np.array([( compPosVar,              0,          0,          0 ),\
                              (          0, compPrevPosVar,          0,          0 ),\
                              (          0,              0, initAngVar,          0 ),\
                              (          0,              0,          0, initAngVar )])
            for i in range(nPts):
                pos = intWidth*(i+0.5)
                if cropRange != None:
                    cPos = mapgraph.get_road_position(curr_street,np.array([pos]))
                    if not is_inbounds(cPos,cropRange[0],cropRange[1]):
                        currDist.logweights[i] = -np.inf
                        continue 
                mu = np.array([[pos], [pos - self.dt*initSpeed], [0], [0]])
                currDist.setComponentLogW(i, logw, mu, sigma)
            currDist.removeUnusedComponents()
        else:
            currDist = GaussianMixtureModel(len(mu),1)
            mu = np.array([[initPos], [initSpeed], [calpha*initPos], [0]])
            sigma = np.array([( initPosVar,            0,          0,          0),\
                              (          0, initSpeedVar,          0,          0),\
                              (          0,            0, initAngVar,          0),\
                              (          0,            0,          0, (0.1)**2)])
            currDist.setComponentLogW(0, 0, np.dot(A,mu), np.dot(A,np.dot(sigma,A.T)))
        return currDist
    

#class MapState3Dynamics(MapDynamics):
#    # Observation parameters
#    v_sigma = None
#    dtheta_sigma = None
#
#    # State model parameters
#    lin_acc_sigma = None
#    ang_acc_sigma = None
#    k_theta = None
#    d_theta = None
#
#    def __init__(self,dt,dt_1,dt_2,v_sigma,dtheta_sigma,lin_acc_sigma,ang_acc_sigma,k_theta,d_theta):
#        super(MapState3Dynamics, self).__init__(6,2)
#        self.setParameters(dt,dt_1,dt_2,v_sigma,dtheta_sigma,lin_acc_sigma,ang_acc_sigma,k_theta,d_theta)
#
#    def fit(self,mapgraph,sequences):
#        D = []
#        Dobs = []
#        for (obs,states) in sequences:
#            for (N,((t,curr_data_dt,yt),currState)) in enumerate(izip(obs,states)):
#                self.setParameters(dt = curr_data_dt,dt_1 = self.dt,dt_2 = self.dt_1)
#                if N == 0 or currState['street'] != prevState['street']:
#                    prevState = currState
#                    continue
##                if mapgraph.node[currState['street']]['length'] - currState['state'][0] < 10.0/1000.0:
##                    continue
#                (obsMu,obsSigma) = self.observation_distribution(mapgraph,currState['street'], currState['state'])
#                Dobs.append(yt - obsMu)
#                (mu,Sigma) = self.state_transition_distribution(mapgraph, currState['street'], prevState['street'], prevState['state'])
#                D.append((mu - currState['state'])/self.dt**2)
#                prevState = currState
#                
#        errMat = np.array(D)
#        errSigma = np.cov(errMat.reshape(errMat.shape[0],errMat.shape[1]), rowvar=0)
#        print 'State transition std deviations',np.sqrt(errSigma[(0,3),(0,3)])
#        print np.linalg.cholesky(errSigma[(0,0,3,3),(0,3,0,3)].reshape([2,2]))
#
#        errMat = np.array(Dobs)
#        errSigma = np.cov(errMat.reshape(errMat.shape[0],errMat.shape[1]), rowvar=0)
#        print 'Observation std deviations',np.sqrt(errSigma[(0,1),(0,1)])
# 
#    def convert_posorient_sequence(self,posorients):
#        states = []
#        (prev2Pos,prev2Orient) = posorients[0]
#        (prevPos,prevOrient) = posorients[1]
#        for (pos,orient) in posorients[2:]:
#            states.append(np.array((pos,prevPos,prev2Pos,orient,prevOrient,prev2Orient)).reshape(self.state_dim,1))
#            prevPos, prevOrient = pos, orient 
#        return states
#
#    def init_state_from_speed(self,speed):
#        return np.array([[0],[-speed*self.dt],[-2*speed*self.dt],[0],[0],[0]])
#
#    def get_state_vel_matrix(self):
#        return np.array([[1,0,0,0,0,0],[1.0/self.dt,-1.0/self.dt,0,0,0,0],[0,0,0,1,0,0],[0,0,0,1.0/self.dt,-1.0/self.dt,0]])
#
#    def get_canonical_state_transform(self,mapgraph,vt):
#        return (np.array([[1,0,0,0,0,0],[0,0,0,1,0,0]]),np.array([[0],[0]]))
#
#    def setParameters(self,dt = None,dt_1 = None,dt_2 = None,v_sigma = None, dtheta_sigma = None, lin_acc_sigma = None, ang_acc_sigma = None, k_theta = None, d_theta = None):
#        if dt != None:
#            self.dt = dt 
#        if dt_1 != None:
#            self.dt_1 = dt_1 
#        if dt_2 != None:
#            self.dt_2 = dt_2 
#        if v_sigma != None:
#            self.v_sigma = v_sigma 
#        if dtheta_sigma != None:
#            self.dtheta_sigma = dtheta_sigma
#        if lin_acc_sigma != None:
#            self.lin_acc_sigma = lin_acc_sigma 
#        if ang_acc_sigma != None:
#            self.ang_acc_sigma = ang_acc_sigma
#        if k_theta != None:
#            self.k_theta = k_theta
#        if d_theta != None:
#            self.d_theta = d_theta
#
#        dtr = (self.dt/self.dt_1)
#        dtr2 = (self.dt/self.dt_2)
#        dt2 = self.dt**2
##        t1 = 1 + dtr*(1-self.dt*self.d_theta) - dt2*self.k_theta
##        t2 = -dtr*(1-self.dt*self.d_theta)
#        d_1fact = 1 + dtr + dtr**2 
#        d_2fact = -dtr*(1 + dtr + dtr2) 
#        d_3fact = dtr*dtr2
#        d_1angfact = 1 + dtr + dtr**2 - dt2*self.k_theta - self.dt*dtr*self.d_theta
#        d_2angfact = -dtr*(1 + dtr + dtr2 - self.dt*self.d_theta)
#        d_1springfact = -dt2*self.k_theta - (dt2/self.dt_1)*self.d_theta
#        d_2springfact = (dt2/self.dt_1)*self.d_theta
#        self.A_s = np.array([[ d_1fact, d_2fact, d_3fact,  0, 0, 0 ], \
#                             [       1,       0,       0,  0, 0, 0 ], \
#                             [       0,       1,       0,  0, 0, 0 ], \
#                             [       0,       0,       0,  d_1angfact, d_2angfact, d_3fact ],\
#                             [       0,       0,       0,           1,          0,       0 ],\
#                             [       0,       0,       0,           0,          1,       0 ]])
#        zeroSigma2 = 0.000001**2
#        self.Sigma_s = np.array([[ (dt2*self.lin_acc_sigma)**2,           0,           0,                           0,           0,           0 ], \
#                                 [                           0,  zeroSigma2,           0,                           0,           0,           0 ], \
#                                 [                           0,           0,  zeroSigma2,                           0,           0,           0 ], \
#                                 [                           0,           0,           0, (dt2*self.ang_acc_sigma)**2,           0,           0 ], \
#                                 [                           0,           0,           0,                           0,  zeroSigma2,           0 ], \
#                                 [                           0,           0,           0,                           0,           0,  zeroSigma2 ]])
#        self.cholSigma_s = np.power(self.Sigma_s,0.5)
#
#        self.A_s0 = self.A_s
#        self.Sigma_s0 = self.Sigma_s
#        self.cholSigma_s0 = self.cholSigma_s
#
#        self.A_y = np.array([[1.0/self.dt,-1.0/self.dt,0,0,0,0],[0,0,0,1.0/self.dt,-1.0/self.dt,0]])
#        self.b_y = np.array([[0],[0]])
#        self.Sigma_y = np.array([(self.v_sigma**2/self.dt,0),(0,self.dtheta_sigma**2/self.dt)])
#        self.logdetSigma_y = np.log(np.linalg.det(self.Sigma_y))
#        self.cholSigma_y = np.linalg.cholesky(self.Sigma_y)
#        self.infSigma_y = np.linalg.inverse(self.Sigma_y)
##        self.logdetSigma_y = 2*np.log(self.v_sigma) + 2*np.log(self.dtheta_sigma)
##        self.cholSigma_y = np.array([(self.v_sigma,0),(0,self.dtheta_sigma)])
##        self.invSigma_y = np.array([(self.v_sigma**-2.0,0),(0,self.dtheta_sigma**-2.0)])
#
#    def observation_distribution(self,mapgraph,vt,st = None,returnChol=False):
#        if returnChol:
#            Sigma = (self.Sigma_y,self.cholSigma_y)
#        else:
#            Sigma = self.Sigma_y
#        if st == None:
#            return (self.A_y,self.b_y,Sigma)
#        else:
#            return (np.dot(self.A_y,st) + self.b_y,Sigma)
#
#    def state_transition_distribution(self, mapgraph, vt, vt_1, st_1=None,returnChol=False):
#        aFactor = self.k_theta * self.dt**2 * mapgraph.node[vt_1]['curve_alpha']
#        AupdMat = np.array([(1,0,0,0,0,0),(0,1,0,0,0,0),(0,0,1,0,0,0),(aFactor,0,0,1,0,0),(0,0,0,0,1,0),(0,0,0,0,0,1)])
#        if vt == vt_1:
#            Aref = self.A_s
#            b = np.zeros((self.A_s.shape[0], 1))
#            if returnChol:
#                Sigma = (self.Sigma_s,self.cholSigma_s)
#            else:
#                Sigma = self.Sigma_s
#        else:
#            clen = mapgraph[vt_1][vt]['transition_distance']
#            angle = mapgraph[vt_1][vt]['angle']
#            Aref = self.A_s0
#            b = np.array([[-clen], [-clen], [-clen], [(self.dt**2*self.k_theta - 1.0)*angle], [-angle], [-angle]])
#            if returnChol:
#                Sigma = (self.Sigma_s0,self.cholSigma_s0)
#            else:
#                Sigma = self.Sigma_s0
#        A = np.dot(AupdMat,Aref)
#        if st_1 == None:
#            return (A, b, Sigma)
#        else:
#            mu = np.dot(A,st_1) + b
#            return (mu,Sigma)
#
#    def get_initial_distribution(self,mapgraph,curr_street,initPos,initPosVar,initSpeed,initSpeedVar):
#        A = np.array(((1,0,0,0,0,0),(1,-self.dt,-self.dt**2,0,0,0),(1,-2*self.dt,-4*self.dt**2,0,0,0), \
#                      (0,0,0,1,0,0),(0,0,0,1,-self.dt,-self.dt**2),(0,0,0,1,-2*self.dt,-4*self.dt**2)))
#        initAngVar = (0.001)**2
#        initAngVelVar = (0.1)**2
#        accVar = 0.1^2*initSpeedVar
#        angAccVar = 0.1^2*initAngVelVar
#        if np.isinf(initPosVar):
#            clen = mapgraph.node[curr_street]['length']
#            calpha = mapgraph.node[curr_street]['curve_alpha']
#            nCompsPerKm = 200 # 100 components per kilometer
#            nPts = int(np.round(nCompsPerKm*clen)) 
#            currDist = GaussianMixtureModel(self.state_dim,nPts+2)
#            intWidth = 1.0/(nPts+1.0)
#            compPosVar = (2*clen*intWidth)**2
#
#            for i in range(nPts):
#                mu = np.array([       [(1.0+i)*intWidth*clen],        [initSpeed], [0], \
#                               [calpha*(1.0+i)*intWidth*clen], [initSpeed*calpha], [0]])
#                sigma = np.array([( compPosVar,            0,          0,          0,             0,         0 ),\
#                                  (          0, initSpeedVar,          0,          0,             0,         0 ),\
#                                  (          0,            0,     accVar,          0,             0,         0 ),\
#                                  (          0,            0,          0, initAngVar,             0,         0 ),\
#                                  (          0,            0,          0,          0, initAngVelVar,         0 ),
#                                  (          0,            0,          0,          0,             0, angAccVar ) ])
#                currDist.setComponentLogW(i, -np.log(nPts+2), np.dot(A,mu), np.dot(A,np.dot(sigma,A.T)))
#
#            mu = np.array([[0], [initSpeed], [0], [0], [initSpeed*calpha], [0]])
#            sigma = np.array([( 0.5**2*compPosVar,            0,          0,          0,             0,         0 ),\
#                              (          0, initSpeedVar,          0,          0,             0,         0 ),\
#                              (          0,            0,     accVar,          0,             0,         0 ),\
#                              (          0,            0,          0, initAngVar,             0,         0 ),\
#                              (          0,            0,          0,          0, initAngVelVar,         0 ),
#                              (          0,            0,          0,          0,             0, angAccVar ) ])
#            currDist.setComponentLogW(nPts, -np.log(nPts+2), np.dot(A,mu), np.dot(A,np.dot(sigma,A.T)))
#
#            mu = np.array([[clen], [initSpeed], [calpha*clen], [initSpeed*calpha]])
#            sigma = np.array([( 0.5**2*compPosVar,            0,          0,          0,             0,         0 ),\
#                              (          0, initSpeedVar,          0,          0,             0,         0 ),\
#                              (          0,            0,     accVar,          0,             0,         0 ),\
#                              (          0,            0,          0, initAngVar,             0,         0 ),\
#                              (          0,            0,          0,          0, initAngVelVar,         0 ),
#                              (          0,            0,          0,          0,             0, angAccVar ) ])
#            currDist.setComponentLogW(nPts+1, -np.log(nPts+2), np.dot(A,mu), np.dot(A,np.dot(sigma,A.T)))
#        else:
#            currDist = GaussianMixtureModel(len(mu),1)
#            mu = np.array([[initPos], [initSpeed], [0], [calpha*initPos], [0], [0]])
#            sigma = np.array([( initPosVar,            0,          0,          0,             0,         0 ),\
#                              (          0, initSpeedVar,          0,          0,             0,         0 ),\
#                              (          0,            0,     accVar,          0,             0,         0 ),\
#                              (          0,            0,          0, initAngVar,             0,         0 ),\
#                              (          0,            0,          0,          0, initAngVelVar,         0 ),
#                              (          0,            0,          0,          0,             0, angAccVar ) ])
#            currDist.setComponentLogW(0, 0, np.dot(A,mu), np.dot(A,np.dot(sigma,A.T)))
#        return currDist

class MapGMMDistribution:
    logV = None
    Theta = None
    Info = None
    
    def __init__(self):
        self.logV = dict()
        self.Theta = dict()
        self.Info = dict()
        
        
    def update(self,other):
        """ Update the distribution based on information in another posterior """ 
        self.logV.update(other.logV)
        self.Theta.update(other.Theta)
        self.Info.update(other.Info)
        
    def extract(self,localStreets):
        """ Extract distribution information over just a subset of streets """ 
        ret = MapGMMDistribution()
        ret.Theta = dict((k, self.Theta[k]) for k in localStreets)
        ret.logV = dict((k, self.logV[k]) for k in localStreets)
        ret.Info = dict((k, self.Info[k]) for k in localStreets)
        return ret
    
    def extractData(self,localStreets):
        rTheta = [self.Theta[k] for k in localStreets]
        rlogV = [self.logV[k] for k in localStreets]
        rInfo = [self.Info[k] for k in localStreets]
        return (rTheta,rlogV,rInfo)
    
    def updateData(self,localStreets,data):
        self.Theta.update(izip(localStreets,data[0]))
        self.logV.update(izip(localStreets,data[1]))
        self.Info.update(izip(localStreets,data[2]))

    def computeLogVSum(self,mapgraph,localStreets = None):
        if localStreets == None:
            localStreets = mapgraph.nodes_iter()
            valArray = np.empty((len(mapgraph),))
        else:
            valArray = np.empty((len(localStreets),))
        for (k,v) in enumerate(localStreets):
            if v == SINK_ROAD:
                valArray[k] = -np.inf
            else:
                valArray[k] = self.logV[v]
        return logsumexp(valArray,0)

    def setUniformDist(self,mapgraph,mapdynamics,localStreets,initSpeed,initSpeedVar,cropRange = None):
        """ Sets the distribution to be uniform over street positions """
        if localStreets == None:
            localStreets = mapgraph.nodes_iter()
        for curr_street in localStreets:
            if curr_street == SINK_ROAD:
                continue
            clen = mapgraph.node[curr_street]['length']
            self.Theta[curr_street] = mapdynamics.get_initial_distribution(mapgraph,curr_street,0,np.inf,initSpeed,initSpeedVar,cropRange=cropRange)
            self.logV[curr_street] = np.log(clen) - np.log(mapgraph.totalLength)
            if cropRange != None:
                tmpVal = self.Theta[curr_street].normalizeWeights() 
                if not np.isfinite(tmpVal):
                    self.Theta[curr_street] = None
                    self.logV[curr_street] = -np.inf
                else:
                    self.logV[curr_street] += tmpVal
            self.Info[curr_street] = dict()
            
    def setStartStreetDist(self,mapgraph,mapdynamics,localStreets,initPosVar,initSpeed,initSpeedVar):
        """ Sets the distribution to assume the beginning of some street """
        if localStreets == None:
            localStreets = mapgraph.nodes_iter()
        for curr_street in localStreets:
            if curr_street == SINK_ROAD:
                continue
            self.Theta[curr_street] = mapdynamics.get_initial_distribution(mapgraph,curr_street,0,initPosVar,initSpeed,initSpeedVar)
            self.logV[curr_street] = -np.log(np.log(len(mapgraph)))
            self.Info[curr_street] = dict()

    def simplifyDists(self,mapgraph,mapdynamics,localKeys = None,natThresh = 0.001):
        maxCompsPerKm = 100 # maxnumber of GMM components per km before engaging simplifcation 
        targetCompsPerKm = 1 # number of components per km allow
#        natThresh = SIMPLIFY_KLTHRESH # The maximum KL divergence that's allowed when simplifying

        if localKeys == None:
            localKeys = mapgraph.nodes()

        for vt in localKeys:
            if vt == SINK_ROAD:
                continue
            curr_startt = time.time()
            compThresh = np.maximum(3,int(np.ceil(maxCompsPerKm*mapgraph.node[vt]['length'])))
            if np.isfinite(self.logV[vt]) and self.Theta[vt].numComps() > compThresh:
                prevNum = self.Theta[vt].numComps()
                newErr = self.Theta[vt].simplify(natThresh = natThresh,minNumComps = np.ceil(targetCompsPerKm*mapgraph.node[vt]['length']))
#                if prevNum > self.Theta[vt].numComps():
#                    print 'Simplifying {2} removed {0}/{3} components with <= {1} nits of error.'.format(prevNum - self.Theta[vt].numComps(),newErr,vt,prevNum)
            self.Info[vt]['simplify_time'] = time.time() - curr_startt

    def filterDistNew1(self,mapgraph,mapdynamics,obst,localKeys = None):
        # Inference variables
        NptsPerKm = 1000 # number of MC samples per KM to use when sampling
        logDiscardCompThresh = np.log(1.0e-10/len(mapgraph)) # Threshold at which a component is discarded

        stateDim = mapdynamics.state_dim

        normalizeLogV = localKeys == None
        if localKeys == None:
            localKeys = dict((vt,n) for (n,vt) in enumerate(mapgraph.nodes_iter()))
            localPreds = localKeys
        else:
            localKeys = dict((vt,n) for (n,vt) in enumerate(localKeys))
            localPreds = set()
            for vt in localKeys.iterkeys():
                for vt_1 in mapgraph.predecessors(vt):
                    localPreds.add(vt_1)

        postHat = MapGMMDistribution()
        for vt in localKeys:
            if vt == SINK_ROAD:
                continue
            postHat.logV[vt] = 0.0
            postHat.Theta[vt] = GaussianMixtureModel(stateDim)
            
        nKeys = len(localKeys)
        logVhatsumVec = np.empty(nKeys)
        for vt_1 in localPreds:
            if vt_1 == SINK_ROAD:
                continue
            curr_startt = time.time()
            if not np.isfinite(self.logV[vt_1]):
                postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}
                continue
            prevDist = self.Theta[vt_1]

            vt_1len = mapgraph.node[vt_1]['length']
            Nvt_1Pts = int(np.ceil(NptsPerKm*vt_1len))
            sigmaPts = prevDist.drawSamples(Nvt_1Pts)
            logpSigmaPts = mapdynamics.street_transition_logprob(mapgraph,None,vt_1,sigmaPts)
            for (vt,transI) in mapgraph.node[vt_1]['transition_node_index'].iteritems():
                if vt not in localKeys or vt == SINK_ROAD:
                    continue
                vtlen = mapgraph.node[vt]['length']
                (obsA,obsb,(obsSigma,obsCholSigma)) = mapdynamics.observation_distribution(mapgraph,vt,returnChol=True)
                (moveA,moveb,moveSigma) = mapdynamics.state_transition_distribution(mapgraph,vt,vt_1)

                logp = logpSigmaPts[:,transI]

                movePts = moveb + np.dot(moveA,sigmaPts)
                movePtWs = np.exp(logp)
                
                # Update with yt and add each point as a new mixture component to vt.
                for i in range(Nvt_1Pts):
                    nlogw = logp[i]
                    nmu = movePts[:,i].reshape(stateDim,1)
                    nSigma = moveSigma
                    if np.isfinite(nlogw):
                        (logC,nnmu,nnSigma) = gaussian_dist_yxxmu_product_x(obst, obsA, obsb, obsSigma, \
                                                                            nmu, nSigma, \
                                                                            cholSigmay=obsCholSigma)

                        if np.isfinite(logC):
                            cdfVals = stats.norm.cdf(np.array([-streetMargin,vtlen+streetMargin]),loc = nnmu[0], scale = np.sqrt(nnSigma[0,0]))
                            if cdfVals[1] - cdfVals[0] >= streetMarginThresh:
                                postHat.Theta[vt].addComponentLogW(self.logV[vt_1] + nlogw + logC, nnmu, nnSigma)

            postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}

        for (vt,cvtInd) in localKeys.iteritems():
            if vt == SINK_ROAD:
                logVhatsumVec[cvtInd] = -np.inf
                continue
            if postHat.Theta[vt].numComps() > 0:
                clogVhatsum = postHat.Theta[vt].normalizeWeights()
                postHat.Theta[vt].removeUnusedComponents()
                if postHat.Theta[vt].numComps() == 0:
                    clogVhatsum = -np.inf
            else:
                clogVhatsum = -np.inf
            if np.isfinite(clogVhatsum):
                postHat.logV[vt] = clogVhatsum
                logVhatsumVec[cvtInd] = clogVhatsum
            else:
                logVhatsumVec[cvtInd] = -np.inf
                postHat.logV[vt] = -np.inf
                postHat.Theta[vt] = ()
        logVhatsum = logsumexp(logVhatsumVec)
        
        if normalizeLogV:
            # If we're looking at the whole distribution, normalize it.
            for vt in mapgraph.nodes_iter():
                if vt == SINK_ROAD:
                    continue
                postHat.logV[vt] -= logVhatsum
                if np.isfinite(postHat.logV[vt]) and postHat.logV[vt] <= logDiscardCompThresh:
                    postHat.logV[vt] = -np.inf
                    postHat.Theta[vt] = ()            

            return postHat
        else:
            # Otherwise, return the unnormalized distribution and the local 
            # part of the normalization constant
            return (postHat,logVhatsum)
 
    def filterDistNew2(self,mapgraph,mapdynamics,obst,localKeys = None):
        #debugNodes = set(mapgraph.matchNode('0a747abb44*') + mapgraph.matchNode('c6d973f3de*'))
        debugNodes = set() 

        # Inference variables
        Npts = 400 # number of MC samples to use when resorting to sampling
        #alphaThresh = 1e-2 # threshold at which a component is sampled
        compTransProbThresh = 1e-16 
        compKalmanGradThresh = 1e-8
#        logDiscardCompThresh = np.log(1.0e-10/mapgraph.totalLength) # Threshold at which a component is discarded

        stateDim = mapdynamics.state_dim
        
        normalizeLogV = localKeys == None
        if localKeys == None:
            localKeys = dict((vt,n) for (n,vt) in enumerate(mapgraph.nodes_iter()))
            localPreds = localKeys
        else:
            localKeys = dict((vt,n) for (n,vt) in enumerate(localKeys))
            localPreds = set()
            for vt in localKeys.iterkeys():
                for vt_1 in mapgraph.predecessors(vt):
                    localPreds.add(vt_1)

        postHat = MapGMMDistribution()
        for vt in localKeys:
            if vt == SINK_ROAD:
                continue
            postHat.logV[vt] = 0.0
            postHat.Theta[vt] = GaussianMixtureModel(stateDim)
            
        nKeys = len(localKeys)
        logVhatsumVec = np.empty(nKeys)
        for vt_1 in localPreds:
            if vt_1 == SINK_ROAD:
                continue
            curr_startt = time.time()
            if not np.isfinite(self.logV[vt_1]):
                postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}
                continue
            prevDist = self.Theta[vt_1]

            debugFilter = vt_1 in debugNodes and vt_1 in localKeys

            if debugFilter:
                print '{0}: p = {2}, length = {1}'.format(vt_1,mapgraph.node[vt_1]['length'],np.exp(self.logV[vt_1]))
#                print mapgraph.node[vt_1]['transition_distances']

            # accounting variables        
            totalNComps = 0
            nTransComps = 0
            nMidComps = 0
            nDiscMidComps = 0
            
            for i in range(prevDist.numComps()):
                plogw = prevDist.getLogW(i)
                pmu = prevDist.getMu(i)
                pSigma = prevDist.getSigma(i)
                sigmaPts = None
                trans_probs,trans_prob_derivs,vtIndices = mapdynamics.street_transition_probs(mapgraph,vt_1,pmu,Sigma_st_1=pSigma,computeDerivs=True)
                if debugFilter:
                    print '\n ',i,np.exp(plogw),pmu.reshape(stateDim)
                for (vt,transI) in vtIndices.iteritems():
                    if vt != vt_1:
                        totalNComps += 1
                    if vt not in localKeys or vt == SINK_ROAD:
                        continue
                    
                    vtlen = mapgraph.node[vt]['length']
                    (obsA,obsb,(obsSigma,obsCholSigma)) = mapdynamics.observation_distribution(mapgraph,vt,returnChol=True)
                    (moveA,moveb,moveSigma) = mapdynamics.state_transition_distribution(mapgraph,vt,vt_1)
                    trans_prob_vt_1 = trans_probs[0,transI]
                    dtrans_prob_vt_1 = np.sum(np.power(trans_prob_derivs[:,transI],2.0))

                    if debugFilter:
                        print vt,trans_prob_vt_1,dtrans_prob_vt_1

                    if trans_prob_vt_1 > compTransProbThresh and dtrans_prob_vt_1 < compKalmanGradThresh:
                        # This mode will always make the transition
                        cleanAdd = True
                        if vt != vt_1:
                            nTransComps += 1
                        nmu = np.dot(moveA,pmu) + moveb
                        nSigma = moveSigma + np.dot(moveA,np.dot(pSigma,moveA.T))
                        nlogw = plogw + np.log(trans_prob_vt_1)
                    elif trans_prob_vt_1 > compTransProbThresh:
                        cleanAdd = False
                        if vt != vt_1:
                            nMidComps += 1
                        # This mode may or may not make the transition, resort to sampling
                        if sigmaPts == None:
                            sigmaPts = pmu + np.dot(np.linalg.cholesky(pSigma),np.random.randn(stateDim,Npts))
                            sigmaPtInds = mapdynamics.street_state_mask(mapgraph,sigmaPts,vt_1)
                            sigmaPts = (sigmaPts[:,sigmaPtInds]).reshape((stateDim,-1))
                            currNPts = sigmaPts.shape[1]
                            if currNPts > 0:
                                logpSigmaPts = mapdynamics.street_transition_logprob(mapgraph,None,vt_1,sigmaPts)
                    
                        if currNPts == 0:
                            continue
                        logp = logpSigmaPts[:,transI]
                        logsumexpp = logsumexp(logp)

                        if not np.isfinite(logsumexpp):
                            if vt != vt_1:
                                nDiscMidComps += 1
                            continue

                        movePts = moveb + np.dot(moveA,sigmaPts)
                        movePtWs = np.exp(logp - logsumexpp)

                        nlogw = plogw + logsumexpp - np.log(currNPts)
                        nmu = np.dot(movePts,movePtWs.reshape((currNPts,1))).reshape(stateDim,1)
                        nSigma = moveSigma + np.dot(movePts - nmu,np.multiply(movePtWs.reshape((1,currNPts)),movePts - nmu).T)
                    else:
                        # This mode will never make the transition
                        continue


                    if np.isfinite(nlogw):
                        (logC,nnmu,nnSigma) = gaussian_dist_yxxmu_product_x(obst, obsA, obsb, obsSigma, \
                                                                            nmu, nSigma, \
                                                                            cholSigmay=obsCholSigma)
                        nnlogw = self.logV[vt_1] + nlogw + logC
                        if debugFilter:
                            print 'plogw = {0}, nlogw = {1}, nnlogw = {2}'.format(plogw,nlogw,self.logV[vt_1] + nlogw + logC),
                        if np.isfinite(nnlogw):
                            cdfVals = stats.norm.cdf(np.array([-streetMargin,vtlen+streetMargin]),loc = nnmu[0], scale = np.sqrt(nnSigma[0,0]))
                            if cdfVals[1] - cdfVals[0] >= streetMarginThresh:
                                postHat.Theta[vt].addComponentLogW(nnlogw, nnmu, nnSigma)
                            elif debugFilter:
                                print 'dropped cdf'
                        elif debugFilter:
                            print 'dropped inf logC'
                    else:
                        if debugFilter:
                            print 'plogw = {0}, nlogw = {1}'.format(plogw,nlogw)
            if debugFilter:
                print ''

            postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}
#            print 'Trans Comps {3}, Mid Comps: {0}, Discarded: {1}, Total: {2}'.format(nMidComps,nDiscMidComps,totalNComps,nTransComps)

#        print 'Mid Comps: {0}, Discarded: {1}, Total: {2}'.format(nMidComps,nDiscMidComps,totalNComps)

        for (vt,cvtInd) in localKeys.iteritems():
            debugFilter = vt in debugNodes and vt in localKeys
            if vt == SINK_ROAD:
                logVhatsumVec[cvtInd] = -np.inf
                continue

            cThetahat = postHat.Theta[vt]
            if cThetahat.numComps() > 0:
                clogVhatsum = postHat.Theta[vt].normalizeWeights()
            else:
                clogVhatsum = -np.inf

            if debugFilter:
                print 'Normalization {0}: NumComps {1}'.format(vt,cThetahat.numComps())
                print '  logVhatsum = {0}'.format(clogVhatsum)

            if np.isfinite(clogVhatsum):
                postHat.logV[vt] = clogVhatsum
                logVhatsumVec[cvtInd] = clogVhatsum
            else:
                logVhatsumVec[cvtInd] = -np.inf
                postHat.logV[vt] = -np.inf
                postHat.Theta[vt] = ()
        logVhatsum = logsumexp(logVhatsumVec)
        
        if normalizeLogV:
            # If we're looking at the whole distribution, normalize it.
            for vt in mapgraph.nodes_iter():
                if vt == SINK_ROAD:
                    continue
                debugFilter = vt in debugNodes and vt in localKeys
                if debugFilter:
                    print '{0}: logV = {1}, logVhatsum = {2}'.format(vt,postHat.logV[vt],logVhatsum)
                postHat.logV[vt] -= logVhatsum
                logDiscardCompThresh = -50.0 + (np.log(mapgraph.node[vt]['length']) - np.log(mapgraph.totalLength))
                if np.isfinite(postHat.logV[vt]) and postHat.logV[vt] <= logDiscardCompThresh:
                    if debugFilter:
                        print '  logV = {1} < {2}, dropping street'.format(vt,postHat.logV[vt],logDiscardCompThresh)
                    postHat.logV[vt] = -np.inf
                    postHat.Theta[vt] = ()
        # Otherwise, return the unnormalized distribution and the local 
        # part of the normalization constant
        return (postHat,logVhatsum)
        
    def filterDistNew3(self,mapgraph,mapdynamics,obst,localKeys = None):
        #debugNodes = set(mapgraph.matchNode('0a747abb44*') + mapgraph.matchNode('c6d973f3de*'))
        debugNodes = set() 

        # Inference variables
        Npts = 400 # number of MC samples to use when resorting to sampling
        #alphaThresh = 1e-2 # threshold at which a component is sampled
        compTransProbThresh = 1e-16 
        compKalmanGradThresh = 1e-8
#        logDiscardCompThresh = np.log(1.0e-10/mapgraph.totalLength) # Threshold at which a component is discarded

        stateDim = mapdynamics.state_dim
        
        normalizeLogV = localKeys == None
        if localKeys == None:
            localKeys = dict((vt,n) for (n,vt) in enumerate(mapgraph.nodes_iter()))
            localPreds = localKeys
        else:
            localKeys = dict((vt,n) for (n,vt) in enumerate(localKeys))
            localPreds = set()
            for vt in localKeys.iterkeys():
                for vt_1 in mapgraph.predecessors(vt):
                    localPreds.add(vt_1)

        postHat = MapGMMDistribution()
        for vt in localKeys:
            if vt == SINK_ROAD:
                continue
            postHat.logV[vt] = 0.0
            postHat.Theta[vt] = GaussianMixtureModel(stateDim)
            
        nKeys = len(localKeys)
        logVhatsumVec = np.empty(nKeys)
        for vt_1 in localPreds:
            if vt_1 == SINK_ROAD:
                continue
            curr_startt = time.time()
            if not np.isfinite(self.logV[vt_1]):
                postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}
                continue
            prevDist = self.Theta[vt_1]

            debugFilter = vt_1 in debugNodes and vt_1 in localKeys

            if debugFilter:
                print '{0}: p = {2}, length = {1}'.format(vt_1,mapgraph.node[vt_1]['length'],np.exp(self.logV[vt_1]))
#                print mapgraph.node[vt_1]['transition_distances']

            # accounting variables        
            totalNComps = 0
            nTransComps = 0
            nMidComps = 0
            nDiscMidComps = 0
            
            transDists = dict((vt,GaussianMixtureModel(stateDim)) for vt in mapgraph.successors_iter(vt_1))

            for i in range(prevDist.numComps()):
                plogw = prevDist.getLogW(i)
                pmu = prevDist.getMu(i)
                pSigma = prevDist.getSigma(i)
                sigmaPts = None
                trans_probs,trans_prob_derivs,vtIndices = mapdynamics.street_transition_probs(mapgraph,vt_1,pmu,Sigma_st_1=pSigma,computeDerivs=True)
                if debugFilter:
                    print '\n ',i,np.exp(plogw),pmu.reshape(stateDim)
                for (vt,transI) in vtIndices.iteritems():
                    if vt != vt_1:
                        totalNComps += 1
                    if vt not in localKeys or vt == SINK_ROAD:
                        continue
                    
                    vtlen = mapgraph.node[vt]['length']
                    (obsA,obsb,(obsSigma,obsCholSigma)) = mapdynamics.observation_distribution(mapgraph,vt,returnChol=True)
                    (moveA,moveb,moveSigma) = mapdynamics.state_transition_distribution(mapgraph,vt,vt_1)
                    trans_prob_vt_1 = trans_probs[0,transI]
                    dtrans_prob_vt_1 = np.sum(np.power(trans_prob_derivs[:,transI],2.0))

                    if debugFilter:
                        print vt,trans_prob_vt_1,dtrans_prob_vt_1

                    if trans_prob_vt_1 > compTransProbThresh and dtrans_prob_vt_1 < compKalmanGradThresh:
                        # This mode will always make the transition
                        cleanAdd = True
                        if vt != vt_1:
                            nTransComps += 1
                        nmu = np.dot(moveA,pmu) + moveb
                        nSigma = moveSigma + np.dot(moveA,np.dot(pSigma,moveA.T))
                        nlogw = plogw + np.log(trans_prob_vt_1)
                    elif trans_prob_vt_1 > compTransProbThresh:
                        cleanAdd = False
                        if vt != vt_1:
                            nMidComps += 1
                        # This mode may or may not make the transition, resort to sampling
                        if sigmaPts == None:
                            sigmaPts = pmu + np.dot(np.linalg.cholesky(pSigma),np.random.randn(stateDim,Npts))
                            sigmaPtInds = mapdynamics.street_state_mask(mapgraph,sigmaPts,vt_1)
                            sigmaPts = (sigmaPts[:,sigmaPtInds]).reshape((stateDim,-1))
                            currNPts = sigmaPts.shape[1]
                            if currNPts > 0:
                                logpSigmaPts = mapdynamics.street_transition_logprob(mapgraph,None,vt_1,sigmaPts)
                    
                        if currNPts == 0:
                            continue
                        logp = logpSigmaPts[:,transI]
                        logsumexpp = logsumexp(logp)

                        if not np.isfinite(logsumexpp):
                            if vt != vt_1:
                                nDiscMidComps += 1
                            continue

                        movePts = moveb + np.dot(moveA,sigmaPts)
                        movePtWs = np.exp(logp - logsumexpp)

                        nlogw = plogw + logsumexpp - np.log(currNPts)
                        nmu = np.dot(movePts,movePtWs.reshape((currNPts,1))).reshape(stateDim,1)
                        nSigma = moveSigma + np.dot(movePts - nmu,np.multiply(movePtWs.reshape((1,currNPts)),movePts - nmu).T)
                    else:
                        # This mode will never make the transition
                        continue


                    if np.isfinite(nlogw):
                        (logC,nnmu,nnSigma) = gaussian_dist_yxxmu_product_x(obst, obsA, obsb, obsSigma, \
                                                                            nmu, nSigma, \
                                                                            cholSigmay=obsCholSigma)
                        nnlogw = self.logV[vt_1] + nlogw + logC
                        if debugFilter:
                            print 'plogw = {0}, nlogw = {1}, nnlogw = {2}'.format(plogw,nlogw,self.logV[vt_1] + nlogw + logC),
                        if np.isfinite(nnlogw):
                            cdfVals = stats.norm.cdf(np.array([-streetMargin,vtlen+streetMargin]),loc = nnmu[0], scale = np.sqrt(nnSigma[0,0]))
                            if cdfVals[1] - cdfVals[0] >= streetMarginThresh:
                                if cleanAdd or vt == vt_1:
                                    if debugFilter:
                                        print 'clean add'
                                    postHat.Theta[vt].addComponentLogW(nnlogw, nnmu, nnSigma)
                                else:
                                    if debugFilter:
                                        print 'mixed add'
                                    transDists[vt].addComponentLogW(nnlogw, nnmu, nnSigma)
                            elif debugFilter:
                                print 'dropped cdf'
                        elif debugFilter:
                            print 'dropped inf logC'
                    else:
                        if debugFilter:
                            print 'plogw = {0}, nlogw = {1}'.format(plogw,nlogw)
            if debugFilter:
                print ''

            for vt in mapgraph.successors_iter(vt_1):
                if vt == SINK_ROAD or transDists[vt].numComps() == 0:
                    continue
                if vt == vt_1:
                    postHat.Theta[vt].addMixtureLogW(0, transDists[vt])
                else:
                    cvtlogW = transDists[vt].normalizeWeights()
                    (nnmu,nnSigma) = transDists[vt].computeMeanCovar()
                    postHat.Theta[vt].addComponentLogW(cvtlogW, nnmu, nnSigma)

            postHat.Info[vt_1] = {'filter_time':(time.time() - curr_startt)}
#            print 'Trans Comps {3}, Mid Comps: {0}, Discarded: {1}, Total: {2}'.format(nMidComps,nDiscMidComps,totalNComps,nTransComps)

#        print 'Mid Comps: {0}, Discarded: {1}, Total: {2}'.format(nMidComps,nDiscMidComps,totalNComps)

        for (vt,cvtInd) in localKeys.iteritems():
            debugFilter = vt in debugNodes and vt in localKeys
            if vt == SINK_ROAD:
                logVhatsumVec[cvtInd] = -np.inf
                continue

            cThetahat = postHat.Theta[vt]
            if cThetahat.numComps() > 0:
                clogVhatsum = postHat.Theta[vt].normalizeWeights()
            else:
                clogVhatsum = -np.inf

            if debugFilter:
                print 'Normalization {0}: NumComps {1}'.format(vt,cThetahat.numComps())
                print '  logVhatsum = {0}'.format(clogVhatsum)

            if np.isfinite(clogVhatsum):
                postHat.logV[vt] = clogVhatsum
                logVhatsumVec[cvtInd] = clogVhatsum
            else:
                logVhatsumVec[cvtInd] = -np.inf
                postHat.logV[vt] = -np.inf
                postHat.Theta[vt] = ()
        logVhatsum = logsumexp(logVhatsumVec)
        
        if normalizeLogV:
            # If we're looking at the whole distribution, normalize it.
            for vt in mapgraph.nodes_iter():
                if vt == SINK_ROAD:
                    continue
                debugFilter = vt in debugNodes and vt in localKeys
                if debugFilter:
                    print '{0}: logV = {1}, logVhatsum = {2}'.format(vt,postHat.logV[vt],logVhatsum)
                postHat.logV[vt] -= logVhatsum
                logDiscardCompThresh = -50.0 + (np.log(mapgraph.node[vt]['length']) - np.log(mapgraph.totalLength))
                if np.isfinite(postHat.logV[vt]) and postHat.logV[vt] <= logDiscardCompThresh:
                    if debugFilter:
                        print '  logV = {1} < {2}, dropping street'.format(vt,postHat.logV[vt],logDiscardCompThresh)
                    postHat.logV[vt] = -np.inf
                    postHat.Theta[vt] = ()
        # Otherwise, return the unnormalized distribution and the local 
        # part of the normalization constant
        return (postHat,logVhatsum)

    def filterDist(self,mapgraph,mapdynamics,obst,localKeys = None,filterVer = 0,simplifyThresh = 0):
        if filterVer == 1:
            ret =  self.filterDistNew1(mapgraph, mapdynamics, obst, localKeys)
        elif filterVer == 2:
            # v2 propogates multiple sampled mixture components to new roads
            ret =  self.filterDistNew2(mapgraph, mapdynamics, obst, localKeys)
        elif filterVer == 3:
            # v3 propogates multiple sampled mixture components to new roads as a single mode
            ret =  self.filterDistNew3(mapgraph, mapdynamics, obst, localKeys)
#        elif filterVer == 4:
#            # v4 propogates multiple sampled mixture components to new roads and simplifys them
#            ret =  self.filterDistNew4(mapgraph, mapdynamics, obst, localKeys)
            
        if simplifyThresh > 0:
            ret[0].simplifyDists(mapgraph, mapdynamics, localKeys, simplifyThresh)

        return ret
    
    def roadSegmentModes(self,mapgraph,mapdynamics,N,segmentLength,nmsRadius = 0,nmsCutoffPct = 0):
        nms = SpatialAngleNMS(N = N,maxCutoff = -np.inf,spatialRad = nmsRadius,angleThresh = np.pi/2.0,cutoffPctThresh = nmsCutoffPct)

        for curr_street in mapgraph.nodes_iter():
            if curr_street == SINK_ROAD:
                continue

            curr_logw = self.logV[curr_street]
            curr_w = np.exp(curr_logw)
            curr_len = mapgraph.node[curr_street]['length']
            
            if curr_w > 0:
                (A,b) = mapdynamics.get_canonical_state_transform(mapgraph,curr_street)
                curr_dist = self.Theta[curr_street]
                trans_curr_dist = curr_dist.transform(A[0,:].reshape([1,A.shape[1]]),b[0])
                
                nParts = int(np.ceil(curr_len/segmentLength))
                cdfPts = np.linspace(0,curr_len,nParts+1)
                cdfVals = np.zeros(nParts)
                
                allCDFVals = trans_curr_dist.cdf(cdfPts)
                allCDFVals[0] = 0
                allCDFVals[-1] = 1.0
                cdfVals = curr_w*(allCDFVals[1:nParts+1] - allCDFVals[0:nParts]) 

                roadPts = mapgraph.get_road_position(curr_street,cdfPts,0)
                for i in range(nParts):
                    street_pos = 0.5*(cdfPts[i] + cdfPts[i+1])
                    angI = mapgraph.node[curr_street]['curve_alpha']*street_pos + mapgraph.node[curr_street]['curve_beta']
                    elemData = {'street':curr_street,'street_pos':street_pos}
                    nms.add(cdfVals[i],0.5*(roadPts[:,i] + roadPts[:,i+1]),angI,elemData)
            

        nmsList = nms.getModes()
        retList = []
        for (val,valInd,pos,ang,data) in nmsList:
            curr_street = data['street']
            (A,b) = mapdynamics.get_canonical_state_transform(mapgraph,curr_street)
            curr_dist = self.Theta[curr_street]
            trans_curr_dist = curr_dist.transform(A,b)
            x0 = np.array([ [data['street_pos']], [ang] ])
            x1 = trans_curr_dist.findMaximum(x0)
            retList.append((val,curr_street,mapgraph.get_road_position(curr_street,np.array([x1[0]])),x1[1]))

        return retList

    def displayPosterior(self,mapgraph,mapdynamics,fig,ax = None,gtX = None,gtPath = None, odomPosPath = None, odomIntPos = None, modes = None):
        dispModes = False

        if ax == None:
            ax = fig.add_subplot(111)

        gtPos = None
        gtPosPath = None
        if gtX != None:
            if 'position' in gtX:
                gtPos = gtX['position']
            elif 'latlon' in gtX:
                gtPos = mercatorProj(gtX['latlon'],mapgraph.graph['mercator_scale']) - mapgraph.graph['position_offset']
            elif 'street' in gtX and 'state' in gtX:
                gtStreet = gtX['street']
                gtState = gtX['state']
                gtPos = mapgraph.get_road_position(gtStreet,gtState[0],BIDI_OFFSET)
            else:
                assert(False)
        if gtPath != None:
            gtPosPath = [];
            for cgt in gtPath:
                if 'position' in cgt:
                    cgtPos = cgt['position']
                elif 'latlon' in cgt:
                    cgtPos = mercatorProj(cgt['latlon'],mapgraph.graph['mercator_scale']) - mapgraph.graph['position_offset']
                elif 'street' in cgt and 'state' in gtX:
                    gtStreet = cgt['street']
                    gtState = cgt['state']
                    cgtPos = mapgraph.get_road_position(gtStreet,gtState[0],0)
                else:
                    assert(False)
                gtPosPath.append(cgtPos.reshape(2))


        #mapgraph.display(fig,ax,gtPos = gtPos,gtPosPath = gtPosPath,odomPosPath = odomPosPath)
        mapgraph.display(fig,ax,gtPos = gtPos,gtPosPath = gtPosPath)

        ax_xlim = ax.get_xlim();
        ax_ylim = ax.get_ylim();
        
        min_logw = np.inf
        max_logw = -np.inf
        for curr_logw in self.logV.itervalues():
            if np.isfinite(curr_logw):
                min_logw = np.minimum(min_logw,curr_logw)
                max_logw = np.maximum(max_logw,curr_logw)
        if max_logw == min_logw:
            logw_scale = 0.0
            logw_off = 1.0
        else:
            logw_scale = 1.0/(max_logw - min_logw)
            logw_off = -min_logw*logw_scale

        colMap = cm.get_cmap('jet')
        #fig.colorbar(cm.ScalarMappable(colors.Normalize(0,1),colMap),ax)
        ellOffsets = []
        ellWidths = []
        ellHeights = []
        ellAngles = []
        colours = []
        dispLines = []
        dispLineColors = []
        dispPaths = []
        dispPathColors = []
        EglobalHeading = 0

        segmentLength = 10.0/1000.0
        lineWidth = DIST_LINE_WIDTH

        if dispModes and modes == None:
            # Extract no more than 20 modes
            nmsMaxModes = 20
            # A point is considered nearby if it is within a distance of nmsRadius and oriented within nmsAngThresh
            nmsRadius = 10.0*segmentLength
            nmsAngThresh = np.pi/2.0
            # All modes must have a value of at least nmsCutoffPct*maxModeValue
            nmsCutoffPct = 0.1
            # All modes must have at least probability of nmsMinValue
            nmsMinValue = 1e-5
            
            nms = SpatialAngleNMS(N = nmsMaxModes,maxCutoff = nmsMinValue,spatialRad = nmsRadius,angleThresh = nmsAngThresh,cutoffPctThresh = nmsCutoffPct)

        for curr_street in mapgraph.nodes_iter():
            if curr_street == SINK_ROAD:
                continue
            if not is_intersection(ax_xlim,ax_ylim,mapgraph.node[curr_street]['xlim'],mapgraph.node[curr_street]['ylim']):
                continue
            
            curr_logw = self.logV[curr_street]
            curr_w = np.exp(curr_logw)
            curr_len = mapgraph.node[curr_street]['length']
            
            if curr_w > 0:
                (A,b) = mapdynamics.get_canonical_state_transform(mapgraph,curr_street)
                curr_dist = self.Theta[curr_street]
                trans_curr_dist = curr_dist.transform(A[0,:].reshape([1,A.shape[1]]),b[0])
                
                nParts = int(np.ceil(curr_len/segmentLength))
                cdfPts = np.linspace(0,curr_len,nParts+1)
                cdfVals = np.zeros(nParts)
                
                allCDFVals = trans_curr_dist.cdf(cdfPts)
                allCDFVals[0] = 0
                allCDFVals[-1] = 1.0

                cdfVals = curr_w*(allCDFVals[1:nParts+1] - allCDFVals[0:nParts])
                if dispModes and modes == None:
                    roadPts = mapgraph.get_road_position(curr_street,cdfPts,0)
                    for i in range(nParts):
                        street_pos = 0.5*(cdfPts[i] + cdfPts[i+1])
                        angI = mapgraph.node[curr_street]['curve_alpha']*street_pos + mapgraph.node[curr_street]['curve_beta']
                        elemData = {'street':curr_street,'street_pos':street_pos}
                        nms.add(cdfVals[i],0.5*(roadPts[:,i] + roadPts[:,i+1]),angI,elemData)

                dispScale = 0.05*mapgraph.totalLength/(curr_len/nParts)

                alphaMin = 0.01
                roadOffset = 0.49*lineWidth 
                if mapgraph.node[curr_street]['type'] == 'line':
                    roadPts = mapgraph.get_road_position(curr_street,cdfPts,roadOffset)
                    for i in range(nParts):
                        if cdfVals[i] > 0:
                            cCDFVal = np.minimum(1.0,dispScale*float(cdfVals[i]))
                            dispLines.append((roadPts[:,i],roadPts[:,i+1]))
                            cCol = colMap(cCDFVal)
                            ncCol = (cCol[0],cCol[1],cCol[2],np.maximum(alphaMin,np.minimum(1.0,np.power(cCDFVal,1.0/3.0))))
                            dispLineColors.append(ncCol);
                else:
                    startTheta = mapgraph.node[curr_street]['start_theta']
                    endTheta = mapgraph.node[curr_street]['end_theta']
                    r = mapgraph.node[curr_street]['radius']
                    C = mapgraph.node[curr_street]['center']
                    length = mapgraph.node[curr_street]['length']
                    
                    thetaPts = startTheta + ((endTheta - startTheta)/length)*cdfPts
                    for i in range(nParts):
                        if cdfVals[i] > 0:
                            cCDFVal = np.minimum(1.0,dispScale*float(cdfVals[i]))
                            transArcPath = get_circarc(r,C,thetaPts[i],thetaPts[i+1],roadOffset)
                            dispPaths.append(transArcPath)
                            cCol = colMap(cCDFVal)
                            ncCol = (cCol[0],cCol[1],cCol[2],np.maximum(alphaMin,np.minimum(1.0,np.power(cCDFVal,1.0/3.0))))
                            dispPathColors.append(ncCol);

        dx0 = ax.viewLim.width
        dx1 = ax.bbox.width
        sc = (dx1/dx0)*(72.0/fig.dpi)

        if dispModes:
            if modes == None:
                nmsList = nms.getModes()
    
                topSegments = []
                for (val,valInd,pos,ang,data) in nmsList:
                    curr_street = data['street']
                    (A,b) = mapdynamics.get_canonical_state_transform(mapgraph,curr_street)
                    curr_dist = self.Theta[curr_street]
                    trans_curr_dist = curr_dist.transform(A,b)
                    x0 = np.array([ [data['street_pos']], [ang] ])
                    x1 = trans_curr_dist.findMaximum(x0)
                    topSegments.append((val,curr_street,mapgraph.get_road_position(curr_street,np.array([x1[0]])),x1[1]))
            else:
                topSegments = modes
    
            for (p,curr_street,curr_pos,curr_heading) in topSegments:
                ax.plot(curr_pos[0],curr_pos[1],'*',zorder=102)
                if odomIntPos != None and len(odomIntPos) > 1:
                    heading = np.pi/2.0 - curr_heading
                    odomIntRot = np.array([(np.cos(heading), np.sin(heading)), (-np.sin(heading),np.cos(heading))])
                    odomIntArray = np.array(odomIntPos).T
                    odomIntArray = curr_pos.reshape((2,1)) + np.dot(odomIntRot,odomIntArray - odomIntArray[:,-1].reshape((2,1)))
                    ax.add_collection(PathCollection([Path(odomIntArray.T)],transOffset = ax.transData,linewidths=sc*GTPATH_LINE_WIDTH,edgecolors = 'grey',facecolors = 'none',zorder=101));

        ax.add_collection(PathCollection(dispPaths,transOffset = ax.transData, facecolors = 'none',
                                         linewidths=sc*lineWidth, edgecolors = dispPathColors))
        ax.add_collection(LineCollection(dispLines,transOffset = ax.transData,
                                         linewidths=sc*lineWidth,colors = dispLineColors))          

        return (fig,ax)
        
    def displayModes(self,mapgraph,mapdynamics,fig,ax = None,gtX = None,gtPath = None, odomPosPath = None, odomIntPos = None, debugDisplay = False):
        if ax == None:
            ax = fig.add_subplot(111)

        gtPos = None
        gtPosPath = None
        if gtX != None:
            if 'position' in gtX:
                gtPos = gtX['position']
            elif 'latlon' in gtX:
                gtPos = mercatorProj(gtX['latlon'],mapgraph.graph['mercator_scale']) - mapgraph.graph['position_offset']
            elif 'street' in gtX and 'state' in gtX:
                gtStreet = gtX['street']
                gtState = gtX['state']
                gtPos = mapgraph.get_road_position(gtStreet,gtState[0],BIDI_OFFSET)
            else:
                assert(False)
        if gtPath != None:
            gtPosPath = [];
            for cgt in gtPath:
                if 'position' in cgt:
                    cgtPos = cgt['position']
                elif 'latlon' in cgt:
                    cgtPos = mercatorProj(cgt['latlon'],mapgraph.graph['mercator_scale']) - mapgraph.graph['position_offset']
                elif 'street' in cgt and 'state' in gtX:
                    gtStreet = cgt['street']
                    gtState = cgt['state']
                    cgtPos = mapgraph.get_road_position(gtStreet,gtState[0],0)
                else:
                    assert(False)
                gtPosPath.append(cgtPos.reshape(2))

        mapgraph.display(fig,ax,gtPos = gtPos,gtPosPath = gtPosPath,odomPosPath = odomPosPath, debugDisplay = debugDisplay)

        segmentLength = 5.0/1000.0

        nmsRadius = 10.0*segmentLength
        nmsCutoff = 0.1
        topSegments = self.roadSegmentModes(mapgraph,mapdynamics,20,2.0*segmentLength,nmsRadius,nmsCutoff)
        for (p,curr_street,curr_pos,curr_heading) in topSegments:
            ax.plot(curr_pos[0],curr_pos[1],'*',zorder=102)
            if odomIntPos != None and len(odomIntPos) > 1:
                heading = np.pi/2.0 - curr_heading
                odomIntRot = np.array([(np.cos(heading), np.sin(heading)), (-np.sin(heading),np.cos(heading))])
                odomIntArray = np.array(odomIntPos).T
                odomIntArray = curr_pos.reshape((2,1)) + np.dot(odomIntRot,odomIntArray - odomIntArray[:,-1].reshape((2,1)))
                ax.add_collection(PathCollection([Path(odomIntArray.T)],transOffset = ax.transData,linewidths=sc*GTPATH_LINE_WIDTH,edgecolors = 'grey',facecolors = 'none',zorder=101));

        return (fig,ax)

def compute_derived_quantities(roads,intersections):
    for (cnode,croadData) in roads.iteritems():
        if 'type' not in croadData:
            croadData['type'] = 'line'
        else:
            assert(croadData['type'] == 'line' or croadData['type'] == 'arc')
            
        assert 'osm_type' in croadData
        assert 'osm_kind' in croadData
    
        if 'segment_type' not in croadData:
            croadData['segment_type'] = 'road'

        cstartInt = croadData['start_int']
        cendInt = croadData['end_int']
        startInt = intersections[cstartInt]
        endInt = intersections[cendInt]
        startPos = startInt['position']
        endPos = endInt['position']
        
        if croadData['type'] == 'arc':
            C = croadData['center']
            startTheta = np.arctan2(startPos[1] - C[1],startPos[0] - C[0])
            if startTheta < 0:
                startTheta += 2.0*np.pi
            endTheta = np.arctan2(endPos[1] - C[1],endPos[0] - C[0])
            if endTheta < 0:
                endTheta += 2.0*np.pi
            
            if croadData['turn_direction'] < 0 and endTheta > startTheta:
                startTheta += 2.0*np.pi
            elif croadData['turn_direction'] > 0 and endTheta < startTheta:
                endTheta += 2.0*np.pi

#            if 'start_theta' in croadData:
#                print 'startTheta = {0} (orig {1})'.format(startTheta,croadData['start_theta'])
#            if 'end_theta' in croadData:
#                print 'endTheta = {0} (orig {1})'.format(endTheta,croadData['end_theta'])
            croadData['start_theta'] = startTheta 
            croadData['end_theta'] = endTheta 
                
#            startTheta = croadData['start_theta']
#            endTheta = croadData['end_theta']

            startR = np.sqrt(np.sum(np.power(C - startPos,2.0)))
            endR = np.sqrt(np.sum(np.power(C - endPos,2.0)))
            R = 0.5*(startR + endR)
            if not (np.abs(R - startR) < 0.5/1000.0) or not (np.abs(R - endR) < 0.5/1000.0):
                print '{3}: R = {0}, startR = {1}, endR = {2}'.format(R,startR,endR,cnode)
            #assert(np.abs(R - startR) < 0.5/1000.0)
            #assert(np.abs(R - endR) < 0.5/1000.0)
            croadData['radius'] = R

            arcLen = R*np.abs(startTheta - endTheta)
            croadData['length'] = arcLen
            croadData['start_direction'] = np.sign(endTheta - startTheta)*np.array([-np.sin(startTheta),np.cos(startTheta)])
            croadData['end_direction'] = np.sign(endTheta - startTheta)*np.array([-np.sin(endTheta),np.cos(endTheta)])
            croadData['curve_alpha'] = (endTheta - startTheta)/arcLen
            croadData['direction'] = croadData['start_direction']
            croadData['xlim'] = np.array([np.minimum(startPos[0],endPos[0]), np.maximum(startPos[0],endPos[0])])
            croadData['ylim'] = np.array([np.minimum(startPos[1],endPos[1]), np.maximum(startPos[1],endPos[1])])
            
            if np.abs(np.abs(startTheta-endTheta) - 2.0*np.pi) < np.pi/32.0:
                print cnode, startTheta, endTheta, croadData['turn_direction'], arcLen

        elif croadData['type'] == 'line':
            cdir = endPos - startPos
            clen = np.sqrt(np.sum(np.power(cdir,2)))
            croadData['length'] = clen
            croadData['direction'] = cdir/clen
            croadData['start_direction'] = croadData['direction']
            croadData['end_direction'] = croadData['direction']
            croadData['curve_alpha'] = 0
            croadData['xlim'] = np.array([np.minimum(startPos[0],endPos[0]), np.maximum(startPos[0],endPos[0])])
            croadData['ylim'] = np.array([np.minimum(startPos[1],endPos[1]), np.maximum(startPos[1],endPos[1])])
        croadData['curve_beta'] = np.arctan2(croadData['start_direction'][1],croadData['start_direction'][0])

def compute_intersecting_roads(roads,intersections,directional = True):
    """ For every intersection, determine which roads start or end there. """
    intEdges = dict((k,{'origin_roads':set(),'terminal_roads':set()}) for k in intersections)
    for (cnode,croadData) in roads.iteritems():
        cstartInt = croadData['start_int']
        cendInt = croadData['end_int']
        cOneWay = croadData['oneway']

        if directional:
            if cOneWay >= 0:
                cnodeKey = (cnode,1)
                intEdges[cstartInt]['origin_roads'].add(cnodeKey)
                intEdges[cendInt]['terminal_roads'].add(cnodeKey)
    
            if cOneWay <= 0:
                cnodeKey = (cnode,-1)
                intEdges[cstartInt]['terminal_roads'].add(cnodeKey)
                intEdges[cendInt]['origin_roads'].add(cnodeKey)
        else:        
            cnodeKey = cnode
            if cOneWay >= 0:
                intEdges[cstartInt]['origin_roads'].add(cnodeKey)
                intEdges[cendInt]['terminal_roads'].add(cnodeKey)
    
            if cOneWay <= 0:
                intEdges[cstartInt]['terminal_roads'].add(cnodeKey)
                intEdges[cendInt]['origin_roads'].add(cnodeKey)
    
    return intEdges

def mercatorProj(latlon,scale):
    EARTH_RAD_EQ = 6378.137 # in km
    return np.array([scale*latlon[1]*(np.pi/180.0)*EARTH_RAD_EQ, scale*EARTH_RAD_EQ*np.log(np.tan((90.0 + latlon[0]) * (np.pi/360.0)))])

