import sys
import numpy as np
import scipy.stats as stats
from copy import copy,deepcopy

from Utils import inv_chol,fit_circle_with_endpoints,line_intersection_2d
from MapGraph import MapGraph,compute_derived_quantities,compute_intersecting_roads,mercatorProj

def circlefit_polyline(startI,endI,points,intersections,fitMeanThreshold,fitMaxThreshold,lenThreshold,relLenThreshold):
    if endI - startI == 1:
        A = intersections[points[startI].sha]['position']
        B = intersections[points[endI].sha]['position']
        d = B - A
        normd = np.sqrt(np.sum(np.power(d,2.0)))
        v = d/normd
        return (startI,endI,{'length':normd,'direction':v,'type':'line'},0)

    pathLen = 0
    for i in range(startI,endI):
        cA = intersections[points[i].sha]['position']
        cB = intersections[points[i+1].sha]['position']
        segLen = np.sqrt(np.sum(np.power(cA - cB,2.0)))
        pathLen += segLen

    numPtsPerKilometer = 100.0
    numPts = np.ceil(numPtsPerKilometer*pathLen)
    distInc = pathLen/(numPts + 1.0)
    currD = distInc
    xPts = []
    for i in range(startI,endI):
        cA = intersections[points[i].sha]['position']
        cB = intersections[points[i+1].sha]['position']
        segLen = np.sqrt(np.sum(np.power(cA - cB,2.0)))
        while currD - segLen <= 0.1*distInc:
            ca = currD / segLen
            cx = ca*cA + (1.0-ca)*cB
            xPts.append(cx)
            currD += distInc
        currD = currD - segLen
    x = np.array(xPts).T

    A = intersections[points[startI].sha]['position']
    B = intersections[points[endI].sha]['position']
#    x = np.array([intersections[points[i].sha]['position'] for i in range(startI+1,endI,1)]).T
    
    # Try fitting a line segment first
    d = B - A
    normd = np.sqrt(np.sum(np.power(d,2.0)))
    v = d/normd
    n = np.array([v[1],-v[0]]) 
    d0 = np.dot(n.T,A)
    es = np.dot(n.T,x) - d0
    if np.mean(np.abs(es)) <= 1.25*fitMeanThreshold and np.max(np.abs(es)) <= 1.25*fitMaxThreshold and np.abs(normd - pathLen) < pathLen*relLenThreshold and np.abs(normd - pathLen) < lenThreshold: 
#        print 'Line: {0} -> {2} -> {1}, es = {3}'.format(A,B,x.shape[1],es)
        return (startI,endI,{'length':normd,'direction':v,'type':'line'},np.max(np.abs(es)))

    # Next try fitting a circle
    (C,r,es) = fit_circle_with_endpoints(x, A, B)
    if np.isfinite(r) and np.mean(np.abs(es)) <= fitMeanThreshold and np.max(np.abs(es)) <= fitMaxThreshold: 
        # If the errors are below threshold

        startTheta = np.arctan2(A[1] - C[1],A[0] - C[0])
        if startTheta < 0:
            startTheta += 2.0*np.pi
        endTheta = np.arctan2(B[1] - C[1],B[0] - C[0])
        if endTheta < 0:
            endTheta += 2.0*np.pi
                    
        testTheta = 0.5*(startTheta + endTheta)
        testP1 = (C + r*np.array([np.cos(testTheta),np.sin(testTheta)])).reshape(2,1)
        testP2 = (C + r*np.array([np.cos(testTheta+np.pi),np.sin(testTheta+np.pi)])).reshape(2,1)
        if np.max(np.abs(np.sum(np.power(x - testP1,2.0),0))) > np.max(np.abs(np.sum(np.power(x - testP2,2.0),0))):
            startTheta -= 2.0*np.pi
        arcLen = r*np.abs(startTheta - endTheta)
        if np.abs(arcLen - pathLen) < pathLen*relLenThreshold and np.abs(arcLen - pathLen) < lenThreshold:
            startDir = np.sign(endTheta - startTheta)*np.array([-np.sin(startTheta),np.cos(startTheta)])
            endDir = np.sign(endTheta - startTheta)*np.array([-np.sin(endTheta),np.cos(endTheta)])
            startAngErr = np.dot(startDir,(x[:,0].reshape(2) - A)/np.sqrt(np.sum(np.power(x[:,0].reshape(2) - A,2.0)))) 
            assert(np.isfinite(startAngErr))
            return (startI,endI, \
                    {'start_direction':startDir, 'end_direction':endDir, \
                     'start_theta':startTheta, 'end_theta':endTheta,
                     'radius':r, 'center':C, \
                     'turn_direction':np.sign(endTheta - startTheta), \
                     'length':arcLen, 'type':'arc'},np.max(np.abs(es)))
    return None

def simplify_polyline(startI,endI,points,intersections,maxLen = None):
    # thresholds for simplifying paths
#    fitMeanThreshold = 0.5/1000.0
#    fitMaxThreshold = 1.0/1000.0 # point position error  
    fitMeanThreshold = 0.5/1000.0
    fitMaxThreshold = 1.5/1000.0 # point position error  
#    lenThreshold = 5.0/1000.0 # path length error
    relLenThreshold = 0.01 # %path length error
    lenThreshold = 0.25/1000.0 # path length error

    nPts = endI - startI + 1
    if nPts == 2:
        return [(startI,endI,{},0)]

    if maxLen == None:
        maxLen = nPts-1

    subsegLens = np.empty(nPts-1)
    for cstartI in range(startI,endI):
        cendI = cstartI+1
        A = intersections[points[cstartI].sha]['position']
        B = intersections[points[cendI].sha]['position']
        d = B - A
        subsegLens[cstartI-startI] = np.sqrt(np.sum(np.power(d,2.0)))
         
    segment = None
    segLenCV = 0
    segLenStd = 0
    subsegLenMin = np.inf
    for cLen in range(maxLen,1,-1):
        for cstartI in range(startI,startI + nPts - cLen):
            cendI = cstartI + cLen
            csegment = circlefit_polyline(cstartI,cendI,points,intersections,fitMeanThreshold,fitMaxThreshold,lenThreshold,relLenThreshold)
            if csegment != None:
                cmin = np.min(subsegLens[(cstartI-startI):(cendI-startI)])
                if cmin < subsegLenMin:
                    subsegLenMin = cmin
                    segment = csegment
#                cmean = np.mean(subsegLens[(cstartI-startI):(cendI-startI)])
#                cstd = np.std(subsegLens[(cstartI-startI):(cendI-startI)])
##                if cstd > segLenStd:
##                    segLenStd = cstd
##                    segment = csegment
#                cCV = cstd/cmean
#                if cCV > segLenCV:
#                    segLenCV = cCV
#                    segment = csegment
        if segment != None:
            break

    if segment == None:
        # If we don't find anything, take the longest single segment
        maxLen = 0
        maxLenI = None
        for cstartI in range(startI,endI):
            cendI = cstartI+1
            A = intersections[points[cstartI].sha]['position']
            B = intersections[points[cendI].sha]['position']
            cpathLen = np.sum(np.power(A-B,2.0))
            if cpathLen > maxLen:
                maxLen = cpathLen
                maxLenI = cstartI
        cstartI = maxLenI
        cendI = cstartI+1
        segment = (cstartI,cendI,{},0)
    else:
        cstartI = segment[0]
        cendI = segment[1]

    if cstartI > startI:
        segments = simplify_polyline(startI,cstartI,points,intersections,cLen)
        segments.append(segment)
    else:
        segments = [segment]

    if cendI < endI:
        segments_right = simplify_polyline(cendI,endI,points,intersections,cLen)
        segments.extend(segments_right)
    
    return segments

#def simplify_polyline(startI,endI,points,intersections,maxLen = None):
#    # thresholds for simplifying paths
#    fitMeanThreshold = 2.0/1000.0
#    fitMaxThreshold = 3.0/1000.0 # point position error  
##    lenThreshold = 5.0/1000.0 # path length error
#    relLenThreshold = 0.03 # % path length error
#    lenThreshold = 1.0 # path length error
#
#    nPts = endI - startI + 1
#    if nPts == 2:
#        return [(startI,endI,{},0)]
#
#    if maxLen == None:
#        maxLen = nPts-1
#    else:
#        maxLen = np.minimum(maxLen,nPts-1)
#
#    subsegLens = np.empty(nPts-1)
#    for cstartI in range(startI,endI):
#        cendI = cstartI+1
#        A = intersections[points[cstartI].sha]['position']
#        B = intersections[points[cendI].sha]['position']
#        d = B - A
#        subsegLens[cstartI-startI] = np.sqrt(np.sum(np.power(d,2.0)))
#
#    seedSegI = np.argmin(subsegLens) + startI
#    
#    segment = None
#    totalSegLen = 0
#    segLenCV = 0
##    segLenStd = 0
##    subsegLenMin = np.inf
#    for cLen in range(maxLen,1,-1):
#        rangeStart = np.maximum(seedSegI - cLen,startI)
#        rangeEnd = np.minimum(seedSegI,np.minimum(seedSegI + cLen - 2,startI + nPts - cLen)-1)
##        print 'cLen = {5}, seedSegI = {2}, range = [{0},{1}], start,end = [{6},{7}]'.format(rangeStart,rangeEnd,seedSegI,cstartI,cendI,cLen,startI,endI)
#        for cstartI in range(rangeStart,rangeEnd+1):
#            cendI = cstartI + cLen
##            print 'c = [{3},{4}]'.format(rangeStart,rangeEnd,seedSegI,cstartI,cendI,cLen,startI,endI)
#            assert(cstartI <= seedSegI)
#            assert(cendI >= seedSegI)
#            csegment = circlefit_polyline(cstartI,cendI,points,intersections,fitMeanThreshold,fitMaxThreshold,lenThreshold,relLenThreshold)
#            if csegment != None:
#                cseglen = np.sum(subsegLens[(cstartI-startI):(cendI-startI)])
#                if cseglen > totalSegLen:
#                    totalSegLen = cseglen
#                    segment = csegment
##                cmin = np.min(subsegLens[(cstartI-startI):(cendI-startI)])
##                if cmin < subsegLenMin:
##                    subsegLenMin = cmin
##                    segment = csegment
##                cmean = np.mean(subsegLens[(cstartI-startI):(cendI-startI)])
##                cstd = np.std(subsegLens[(cstartI-startI):(cendI-startI)])
###                if cstd > segLenStd:
###                    segLenStd = cstd
###                    segment = csegment
##                cCV = cstd/cmean
##                if cCV > segLenCV:
##                    segLenCV = cCV
##                    segment = csegment
#        if segment != None:
#            break
#
#    if segment == None:
#        # If we don't find anything, take the longest single segment
#        maxLen = 0
#        maxLenI = None
#        for cstartI in range(startI,endI):
#            cendI = cstartI+1
#            A = intersections[points[cstartI].sha]['position']
#            B = intersections[points[cendI].sha]['position']
#            cpathLen = np.sum(np.power(A-B,2.0))
#            if cpathLen > maxLen:
#                maxLen = cpathLen
#                maxLenI = cstartI
#        cstartI = maxLenI
#        cendI = cstartI+1
#        segment = (cstartI,cendI,{},0)
#    else:
#        cstartI = segment[0]
#        cendI = segment[1]
#
#    if cstartI > startI:
#        segments = simplify_polyline(startI,cstartI,points,intersections,cLen)
#        segments.append(segment)
#    else:
#        segments = [segment]
#
#    if cendI < endI:
#        segments_right = simplify_polyline(cendI,endI,points,intersections,cLen)
#        segments.extend(segments_right)
#    
#    return segments

def road_length(croad,roads,intersections):
#    if 'length' in roads[croad]:
#        return roads[croad]['length']
    start_int = roads[croad]['start_int']
    start_pos = intersections[start_int]['position']
    end_int = roads[croad]['end_int']
    end_pos = intersections[end_int]['position']
    dist = np.sqrt(np.sum(np.power(end_pos-start_pos,2.0)))
    if 'type' not in roads[croad] or roads[croad]['type'] == 'line':
        return dist
    elif dist > 1e-10:
        C = roads[croad]['center']
        startTheta = np.arctan2(start_pos[1] - C[1],start_pos[0] - C[0])
        if startTheta < 0:
            startTheta += 2.0*np.pi
        endTheta = np.arctan2(end_pos[1] - C[1],end_pos[0] - C[0])
        if endTheta < 0:
            endTheta += 2.0*np.pi
            
        if roads[croad]['turn_direction'] < 0 and endTheta > startTheta:
            startTheta += 2.0*np.pi
        elif roads[croad]['turn_direction'] > 0 and endTheta < startTheta:
            endTheta += 2.0*np.pi

        startR = np.sqrt(np.sum(np.power(C - start_pos,2.0)))
        endR = np.sqrt(np.sum(np.power(C - end_pos,2.0)))
        r = 0.5*(startR + endR)
        
        return r*np.abs(startTheta - endTheta)
    else:
        return dist
    
def interp_road(d,croad,roads,intersections,normD = False):
    """ Get the position of a point along a road """
    start_int = roads[croad]['start_int']
    start_pos = intersections[start_int]['position']
    end_int = roads[croad]['end_int']
    end_pos = intersections[end_int]['position']
    if not normD:
        length = road_length(croad,roads,intersections)
    if 'type' not in roads[croad] or roads[croad]['type'] == 'line':
        if normD:
            alpha = d 
        else:
            alpha = d/length
        return (1.0-alpha)*start_pos + alpha*end_pos 
    else:
        C = roads[croad]['center'].reshape(2)
        startR = np.sqrt(np.sum(np.power(C - start_pos,2.0)))
        endR = np.sqrt(np.sum(np.power(C - end_pos,2.0)))
        r = 0.5*(startR + endR)
        startTheta = np.arctan2(start_pos[1] - C[1],start_pos[0] - C[0])
        if startTheta < 0:
            startTheta += 2.0*np.pi
        endTheta = np.arctan2(end_pos[1] - C[1],end_pos[0] - C[0])
        if endTheta < 0:
            endTheta += 2.0*np.pi
            
        if roads[croad]['turn_direction'] < 0 and endTheta > startTheta:
            startTheta += 2.0*np.pi
        elif roads[croad]['turn_direction'] > 0 and endTheta < startTheta:
            endTheta += 2.0*np.pi
#        startTheta = roads[croad]['start_theta']
#        endTheta = roads[croad]['end_theta']
        if normD:
            curr_theta = (1.0-d)*startTheta + d*endTheta
        else:
            curr_theta = startTheta + (endTheta - startTheta)*(d/length)
        return C + r*np.array([np.cos(curr_theta),np.sin(curr_theta)]).reshape(2)

def interp_road_direction(d,croad,roads,intersections,normD = False):
    """ Get the position of a point along a road """
    start_int = roads[croad]['start_int']
    start_pos = intersections[start_int]['position']
    end_int = roads[croad]['end_int']
    end_pos = intersections[end_int]['position']
    length = road_length(croad,roads,intersections)
    if 'type' not in roads[croad] or roads[croad]['type'] == 'line':
        return (end_pos - start_pos)/length
    else:
        C = roads[croad]['center'].reshape(2)
        r = roads[croad]['radius']
        startTheta = np.arctan2(start_pos[1] - C[1],start_pos[0] - C[0])
        if startTheta < 0:
            startTheta += 2.0*np.pi
        endTheta = np.arctan2(end_pos[1] - C[1],end_pos[0] - C[0])
        if endTheta < 0:
            endTheta += 2.0*np.pi
            
        if roads[croad]['turn_direction'] < 0 and endTheta > startTheta:
            startTheta += 2.0*np.pi
        elif roads[croad]['turn_direction'] > 0 and endTheta < startTheta:
            endTheta += 2.0*np.pi
        if normD:
            curr_theta = (1.0-d)*startTheta + d*endTheta
        else:
            curr_theta = startTheta + (endTheta - startTheta)*(d/length)
        return np.sign(endTheta-startTheta)*np.array([-np.sin(curr_theta),np.cos(curr_theta)]).reshape(2)


def backtrack_from_intersection(startInt,startRoad,backLen,roads,intersections,intData,direction,prevRoads = None, depth = None):
    if depth == None:
        depth = 1
    if prevRoads == None:
        prevRoads = set()
    startLen = road_length(startRoad,roads,intersections)
    startLenExit = 0.499*startLen
    if direction > 0:
        assert(roads[startRoad]['start_int'] == startInt)
        t_startInt = roads[startRoad]['end_int']
        roadSet = 'origin_roads'
        recurse = (backLen > startLenExit) 
    else:
        assert(roads[startRoad]['end_int'] == startInt)
        t_startInt = roads[startRoad]['start_int']
        roadSet = 'terminal_roads'
        recurse = (backLen > (startLen - startLenExit)) 
    prevRoads.add(roads[startRoad]['orig_key'])
    if recurse:
        ret = []
        for ptroad in intData[t_startInt][roadSet]:
            if roads[ptroad]['orig_key'] in prevRoads:
                continue
            ret += backtrack_from_intersection(t_startInt,ptroad,backLen - startLen,roads,intersections,intData,direction,prevRoads,depth+1)
        return ret
    else:
        if backLen < 0:
            excessBackLen = np.abs(backLen)
            backLen = 0
        else:
            excessBackLen = 0

        if direction > 0:
            assert(roads[startRoad]['start_int'] == startInt)
            return [(startRoad,backLen,direction,excessBackLen)]
        else:
            assert(roads[startRoad]['end_int'] == startInt)
            return [(startRoad,startLen - backLen,-direction,excessBackLen)]

def add_turn_segments(roads,intersections,fixedRunwayLen = True):
    debugPrintInfo = False
    alwaysSplitFull = True
    
    runwayLenPerRad = (5.0/1000.0)/(np.pi/2.0) # 10m runway for a 90 deg turn 
    runwayLen = 5.0/1000.0 # 10m runway
    minSplitSpacing = -1
    #minSplitSpacing = 1.0/1000.0
    transThresh = np.pi/8.0
    intData = compute_intersecting_roads(roads,intersections,False)

    newTurnSegments = []
    roadSplits = dict()
    turnAngles = dict()
    removeInts = []
    # Go through each intersection and determine the angles between incident streets
    # and decide whether the intersection needs to have turning segments added. 
    for (cint,cintData) in intersections.iteritems():
        origin_roads = intData[cint]['origin_roads']
        terminal_roads = intData[cint]['terminal_roads']

        splitInt = False
        for troad in terminal_roads:
            tlen = road_length(troad,roads,intersections)
            assert(roads[troad]['end_int'] == cint)
            tdir = roads[troad]['end_direction']
            ttheta = np.arctan2(tdir[1],tdir[0])
            for oroad in origin_roads:
                if roads[troad]['orig_key'] != roads[oroad]['orig_key']: # no U-turns
                    olen = road_length(oroad,roads,intersections)
                    assert(roads[oroad]['start_int'] == cint)
                    odir = roads[oroad]['start_direction']
                    otheta = np.arctan2(odir[1],odir[0])
                    cangle = otheta - ttheta
                    if cangle > np.pi:
                        cangle = cangle - 2.0*np.pi
                    elif cangle < -np.pi:
                        cangle = cangle + 2.0*np.pi

                    turnAngles[(troad,oroad)] = cangle

                    if np.abs(cangle) > transThresh:
                        splitInt = True

        if not splitInt:
            continue

        removeInts.append(cint)

        for troad in terminal_roads:
            tlen = road_length(troad,roads,intersections)

            for oroad in origin_roads:
                if roads[troad]['orig_key'] != roads[oroad]['orig_key']: # no U-turns
#                    debugPrintInfo = troad == '1cee1279e963648b9dcbbfd0a9816af76d68fd1d-1' or oroad == '1cee1279e963648b9dcbbfd0a9816af76d68fd1d-1'
#                    debugPrintInfo = troad == '5f2d4e6dd629929a3bf623c11c16701df4a3200f+1' and oroad == '638f5198c8febbfc703f04c0863a649c71bc245e+1'
#                    debugPrintInfo |= troad == '638f5198c8febbfc703f04c0863a649c71bc245e+1'
                    if (not alwaysSplitFull) and np.abs(cangle) < transThresh:
                        continue
                    olen = road_length(oroad,roads,intersections)
                    
                    if not fixedRunwayLen:
                        runwayLen = np.maximum(0.5/1000.0,np.abs(turnAngles[(troad,oroad)])*runwayLenPerRad)
                        
                    endTurn = backtrack_from_intersection(cint,oroad,runwayLen,roads,intersections,intData,1,set([roads[troad]['orig_key']]))
                    
                    if debugPrintInfo:
                        print '{0} (len = {1}) -> {2} (len = {3})'.format(troad,road_length(troad,roads,intersections),oroad,road_length(oroad,roads,intersections))
#                        print 'start',startTurn
#                        print 'end',endTurn

#                    for st in startTurn:
#                        stroad = st[0]
#                        if stroad not in roadSplits:
#                            roadSplits[stroad] = set()
#                        roadSplits[stroad].add(st[1])

                    for et in endTurn:
                        etroad = et[0]
                        if etroad not in roadSplits:
                            roadSplits[etroad] = set()
                        roadSplits[etroad].add(et[1])
                        
                    turnNum = 0
                    for et in endTurn:
                        if debugPrintInfo:
                            print '  end: {0}'.format(et)
                        startTurn = backtrack_from_intersection(cint,troad,runwayLen+et[3],roads,intersections,intData,-1,set([roads[oroad]['orig_key']]))
                        for st in startTurn:
                            if debugPrintInfo:
                                print '  start: {0}'.format(st)
                            stroad = st[0]
                            if stroad not in roadSplits:
                                roadSplits[stroad] = set()
                            roadSplits[stroad].add(st[1])
                            newTurnSegments.append((st,et,troad,oroad,turnNum))
                            turnNum += 1
                            
    
    rmRoads = []
    newRoads = deepcopy(roads)
    newIntersections = deepcopy(intersections)
    splitIntersections = dict()
    saveSegments = set()
    for (croad,splitSet) in roadSplits.iteritems():
        #debugPrintInfo = croad == '29bfe7e99ce6804defaaa83b53c3b33ddecb6b4f.0000-1'
        if debugPrintInfo:
            print 'length = {0}'.format(road_length(croad,roads,intersections))
        splits = [x for x in splitSet]
        splits.sort()
        
        croad_len = road_length(croad,roads,intersections)
        
        assert splits[0] >= 0
        assert splits[-1] <= croad_len 

        prevInt = newRoads[croad]['start_int']
        prevSplit = 0
        
        for (n,csplit) in enumerate(splits):
            currInt = (croad,csplit)
            currRoad = croad + '-s' + str(n)
            if n == 0 and splits[0] == 0:
                saveSegments.add(currRoad)
            if debugPrintInfo:
                print '{2}: Splitting {0} at {1}'.format(croad,csplit,n)
                print currRoad
                print
            currPos = interp_road(csplit,croad,roads,intersections)
            newIntersections[currInt] = {'position':currPos}
            newRoads[currRoad] = {'start_int':prevInt,'end_int':currInt, \
                                  'oneway':newRoads[croad]['oneway'],'type':newRoads[croad]['type'], \
                                  'orig_key':croad, \
                                  'osm_type':newRoads[croad]['osm_type'], 'osm_kind':newRoads[croad]['osm_kind']}
            if csplit - prevSplit < (0.1/1000.0):
                newRoads[currRoad]['type'] = 'line'
            if newRoads[currRoad]['type'] == 'arc':
                newRoads[currRoad]['center'] = newRoads[croad]['center']
                newRoads[currRoad]['turn_direction'] = newRoads[croad]['turn_direction']
                if debugPrintInfo:
                    print road_length(currRoad,newRoads,newIntersections), (csplit - prevSplit)

            splitIntersections[(croad,csplit)] = currInt

            prevInt = currInt
            prevSplit = csplit

        csplit = croad_len
        currInt = newRoads[croad]['end_int']
        currRoad = croad + '-s' + str(len(splits))
        if np.abs(splits[-1] - croad_len) < 1e-10:
            saveSegments.add(currRoad)
        if debugPrintInfo:
            print '{2}: Splitting {0} at end'.format(croad,csplit,len(splits))
            print currRoad
            print
        newRoads[currRoad] = {'start_int':prevInt,'end_int':currInt, \
                              'oneway':newRoads[croad]['oneway'],'type':newRoads[croad]['type'], \
                              'orig_key':croad, \
                              'osm_type':newRoads[croad]['osm_type'], 'osm_kind':newRoads[croad]['osm_kind']}
        if csplit - prevSplit < (0.1/1000.0):
            newRoads[currRoad]['type'] = 'line'
        if newRoads[currRoad]['type'] == 'arc':
            newRoads[currRoad]['center'] = newRoads[croad]['center']
            newRoads[currRoad]['turn_direction'] = newRoads[croad]['turn_direction']
        
        rmRoads.append(croad)
        
    #compute_derived_quantities(newRoads,newIntersections)
    
    for (orig_st,orig_et,ptroad,poroad,pturnNum) in newTurnSegments:
#        debugPrintInfo = ptroad == 'bcacdf6af8d1aaf1838697fd4dcf8224ed0cc66b' and poroad == '638f5198c8febbfc703f04c0863a649c71bc245e'

        st = splitIntersections[(orig_st[0],orig_st[1])]
        et = splitIntersections[(orig_et[0],orig_et[1])]

        startPos = newIntersections[st]['position']
        endPos = newIntersections[et]['position']
        distSq = np.sum(np.power(startPos - endPos,2.0))

        nroadKey = 't-' + ptroad + '-' + poroad + '-{0}'.format(pturnNum)
        if debugPrintInfo:
            debugRoadKey = copy(nroadKey)
            print debugRoadKey
        newRoads[nroadKey] = {'start_int':st,'end_int':et,'oneway':1,'segment_type':'turn','orig_key':nroadKey, \
                              'osm_type':roads[orig_st[0]]['osm_type'], 'osm_kind':roads[orig_st[0]]['osm_kind'] }

        startDir = orig_st[2]*interp_road_direction(orig_st[1],orig_st[0],roads,intersections)
        endDir = orig_et[2]*interp_road_direction(orig_et[1],orig_et[0],roads,intersections)
        startDirTheta = np.arctan2(startDir[1],startDir[0])
        endDirTheta = np.arctan2(endDir[1],endDir[0])
        turnTheta = endDirTheta - startDirTheta
        if turnTheta > np.pi:
            turnTheta = turnTheta - 2.0*np.pi
        elif turnTheta < -np.pi:
            turnTheta = turnTheta + 2.0*np.pi
#        turnTheta = turnAngles[(ptroad,poroad)]
        turn_direction = np.sign(turnTheta)

        targetDir = startDir
#        targetDir = 0.5*(startDir + endDir)
#        targetDir *= (1./np.sqrt(np.sum(np.power(targetDir,2.0))))
        n = turn_direction*np.array([targetDir[1],-targetDir[0]])
        denom = np.dot(startPos - endPos,n)
        if distSq > (0.5/1000.0)**2 and np.abs(turnTheta) > transThresh and np.abs(turnTheta) < (np.pi - transThresh) and np.abs(denom) > 1e-10:
            r = 0.5*distSq/denom
#            if r < 0:
#                print r,denom,turnTheta
            if r > 0:
                C = startPos - r*n

                startTheta = np.arctan2(startPos[1] - C[1],startPos[0] - C[0])
                if startTheta < 0:
                    startTheta += 2.0*np.pi
                endTheta = np.arctan2(endPos[1] - C[1],endPos[0] - C[0])
                if endTheta < 0:
                    endTheta += 2.0*np.pi
                    
                if turn_direction < 0 and endTheta > startTheta:
                    startTheta += 2.0*np.pi
                elif turn_direction > 0 and endTheta < startTheta:
                    endTheta += 2.0*np.pi
        
                newturnLen = r*np.abs(startTheta - endTheta)
                if newturnLen < 5*runwayLen:
#                if True:
                    newRoads[nroadKey]['type'] = 'arc'
                    newRoads[nroadKey]['center'] = C
                    newRoads[nroadKey]['turn_direction'] = turn_direction
#                else:
#                    print 'r = {0}, dist = {1}, denom = {2}, newTurnLen = {3}'.format(r,np.sqrt(distSq),denom,newturnLen)
        if debugPrintInfo:
            print newRoads[nroadKey]

    for croad in rmRoads:
        del newRoads[croad]

    # Remove the old, unused intersections and connecting end roadbits.
    if alwaysSplitFull:
        # FIXME: We should only delete segments which we're certain are unused.  Some of these are obvious but
        # in general this is subtle.  Need to think on it.
        newIntData = compute_intersecting_roads(newRoads,newIntersections,False)
        for cint in removeInts:
            origin_roads = newIntData[cint]['origin_roads']
            terminal_roads = newIntData[cint]['terminal_roads']
            int_roads = origin_roads | terminal_roads
            allDel = True
            for vt in int_roads:
                allDel = allDel and vt not in saveSegments
                if vt in newRoads and vt not in saveSegments:
                    del newRoads[vt]
            if allDel:
                del newIntersections[cint]

#    remove_short_roads(newRoads,newIntersections,2.0/1000.0)
#    remove_short_roads(newRoads,newIntersections,1.0/1000.0)
    remove_short_roads(newRoads,newIntersections,0.5/1000.0)
#    remove_short_roads(newRoads,newIntersections,0.01/1000.0)

    return (newRoads,newIntersections)

def remove_short_roads(roads,intersections,roadLenThresh = 1.0/1000.0):
    numDel = 1
    totalNumDel = 0
    while numDel > 0:
        newintData = compute_intersecting_roads(roads,intersections,False)
        delRoads = []
        for croad in roads:
            newroadLength = road_length(croad,roads,intersections)
            if newroadLength < roadLenThresh:
                start_int = roads[croad]['start_int']
                end_int = roads[croad]['end_int']
                npos = interp_road(0.5,croad,roads,intersections,True)
#                print 'Pruning {0}...'.format(croad),
                nint = 'p-' + croad
#                print 'adding {0}'.format(nint)
                intersections[nint] = {'position':npos}
                newintData[nint] = dict()
                newintData[nint]['origin_roads'] = set()
                newintData[nint]['terminal_roads'] = set()

                all_roads = newintData[start_int]['origin_roads'] | newintData[end_int]['origin_roads'] | \
                            newintData[start_int]['terminal_roads'] | newintData[end_int]['terminal_roads']

                for oroad in all_roads:
                    # FIXME: May need to tweak center position for arcs
                    if roads[oroad]['start_int'] == start_int or roads[oroad]['start_int'] == end_int:
                        roads[oroad]['start_int'] = nint
                        if roads[oroad]['oneway'] >= 0:
                            newintData[nint]['origin_roads'].add(oroad)
                        if roads[oroad]['oneway'] <= 0:
                            newintData[nint]['terminal_roads'].add(oroad)
                    if roads[oroad]['end_int'] == start_int or roads[oroad]['end_int'] == end_int:
                        roads[oroad]['end_int'] = nint 
                        if roads[oroad]['oneway'] <= 0:
                            newintData[nint]['origin_roads'].add(oroad)
                        if roads[oroad]['oneway'] >= 0:
                            newintData[nint]['terminal_roads'].add(oroad)
                newintData[start_int]['origin_roads'].discard(croad)
                newintData[start_int]['terminal_roads'].discard(croad)
                del intersections[start_int]
                del newintData[start_int]
                if start_int != end_int:
                    newintData[end_int]['origin_roads'].discard(croad)
                    newintData[end_int]['terminal_roads'].discard(croad)
                    del intersections[end_int]
                    del newintData[end_int]
                delRoads.append(croad)

        numDel = len(delRoads)
        totalNumDel += numDel

        for croad in delRoads:
            del roads[croad] 
    if totalNumDel > 0:
        print 'Removed {0} roads shorter than {1}'.format(totalNumDel,roadLenThresh)   

def split_twoway_streets(roads,intersections):
    newRoads = {}
    for (cnode,cnodeData) in roads.iteritems():
        if cnodeData['oneway'] >= 0:
            newRoads[cnode + '+1'] = deepcopy(cnodeData)
            newRoads[cnode + '+1']['oneway'] = 1
        if cnodeData['oneway'] <= 0:
            newRoads[cnode + '-1'] = deepcopy(cnodeData)
            newRoads[cnode + '-1']['oneway'] = 1
            newRoads[cnode + '-1']['start_int'] = cnodeData['end_int']
            newRoads[cnode + '-1']['end_int'] = cnodeData['start_int']
            if cnodeData['type'] == 'arc':
                newRoads[cnode + '-1']['turn_direction'] = -cnodeData['turn_direction']
    return (newRoads,intersections)

def build_osm_map(binFile,doSimplify = True,prunePolylineNodes = True, lat_range = None, lon_range = None):
#    doSimplify = False
    import osm_objects
    
    tObj = osm_objects.Objects(True)
    tObj.loadFromFile(binFile)
    tRange = tObj.getMapRange()
#    print 'map range: lat {0} - {1}, lon {2} - {3}'.format(tRange.lat_min,tRange.lat_max,tRange.lon_min,tRange.lon_max)
    if lat_range != None:
        tRange.geo_min.lat,tRange.geo_max.lat = lat_range[0],lat_range[1] 
    if lon_range != None:
        tRange.geo_min.lon,tRange.geo_max.lon = lon_range[0],lon_range[1] 
    tObjs = tObj.getObjectsInRange(tRange)
    
    Npt = 0
    Npoly = 0
    NpolySegs = 0
    ptCoordSum = np.array([0,0]).T

    intersections = {}
    for t in tObjs:
        if t.isPoint():
            tSha = t.sha
            tPt = t.getPoint()
            ptLat = tPt.geo.lat
            ptLon = tPt.geo.lon
            intersections[tSha] = dict()
            ptCoord = np.array([ptLat,ptLon]).T
            intersections[tSha]['latlon'] = ptCoord
            Npt += 1
            ptCoordSum += ptCoord

    ptCoordMean = ptCoordSum/Npt
    projScale = np.cos(ptCoordMean[0]*(np.pi/180.0))
    ptCoordOrigin = mercatorProj(ptCoordMean,projScale)
    for (sha,data) in intersections.iteritems():
        data['position'] = mercatorProj(data['latlon'],projScale) - ptCoordOrigin

    roads = {}
    numObjs = len(tObjs)
    for (tN,t) in enumerate(tObjs):
        sys.stdout.write('\r{0:4.3}% ({1:6} of {2} objects processed)'.format((100.0*tN)/numObjs,tN,numObjs))
        if t.isPolyline():
            tPoly = t.getPolyline()
            if 'oneway' in t.tag:
                owTag = t.tag['oneway']
                if owTag == 'yes' or owTag == 'true':
                    cOneway = 1
                elif owTag == 'no' or owTag == 'false':
                    cOneway = 0
                elif owTag == '-1':
                    cOneway = -1
                else:
                    print owTag,type(owTag)
                    assert(False)
            else:
                cOneway = 0
            
            prunePolyline = False
            for tRef in tPoly.points:
                currSha = tRef.sha
                if currSha not in intersections:
                    prunePolyline = True
                    break
            if prunePolyline:
                continue

            if 'name' in t.tag:
                cName = unicode(t.tag['name'],encoding='utf-8')
                if ('split_middle' not in t.tag or t.tag['split_middle'] == 'true'):
                    nameDisp = True
                else:
                    nameDisp = False
            else:
                cName = t.sha
                nameDisp = False

            Npoly += 1
            NpolySegs += len(tPoly.points)-1

            if doSimplify:
                segments = simplify_polyline(0,len(tPoly.points)-1,tPoly.points,intersections)
                numKept = len(segments)
                assert(segments[0][0] == 0)
                assert(segments[-1][1] == len(tPoly.points)-1)
            else:
                segments = [(i,i+1,{},0) for i in range(len(tPoly.points)-1)]
                numKept = len(segments)

            for (N,(prevInd,nextInd,segInfo,segErr)) in enumerate(segments):
                if N == 0:
                    assert(prevInd == 0)
                elif N > 0:
                    assert(prevInd == segments[N-1][1])
                if N == numKept-1:
                    assert(nextInd == len(tPoly.points)-1)
                prevSha = tPoly.points[prevInd].sha
                nextSha = tPoly.points[nextInd].sha
                if numKept > 1:
                    roadKey = '{0}.{1:04d}'.format(t.sha,N)
                else:
                    roadKey = '{0}'.format(t.sha)
                roads[roadKey] = {'start_int':prevSha,'end_int':nextSha,'oneway':cOneway, \
                                  'name':cName,'nameDisplayHint':nameDisp,'orig_key':roadKey, \
                                  'osm_type':t.type,'osm_kind':t.kind}
                roads[roadKey].update(segInfo)


    print '\n{0} points loaded for {1} polylines with {2} segments, resulting in {3} roads'.format(Npt, Npoly, NpolySegs, len(roads))

    compute_derived_quantities(roads,intersections)
    (roads,intersection) = split_twoway_streets(roads,intersections)
#    remove_short_roads(roads,intersections,3.0/1000.0)
#    (roads,intersections) = add_turn_segments(roads,intersections,False)
    (roads,intersections) = add_turn_segments(roads,intersections,True)
#    (roads,intersections) = add_turn_segments_old(roads,intersections)
    mg = MapGraph(roads,intersections)
    mg.graph['position_offset'] = ptCoordOrigin
    mg.graph['mercator_scale'] = projScale

    return mg

def build_toy_map(mapVersion = 0):
    rdScale = 0.25
    intersections = {'a':{'position':rdScale*np.array([.10,.20]).T},\
                     'b':{'position':rdScale*np.array([.30,.20]).T},\
                     'c':{'position':rdScale*np.array([.40,.00]).T},\
                     'd':{'position':rdScale*np.array([.00,.00]).T},\
                     'e':{'position':rdScale*np.array([.50,.20]).T},\
                     'f':{'position':rdScale*np.array([.55,.40]).T},\
                     'g':{'position':rdScale*np.array([.35,.40]).T},\
                     'h':{'position':rdScale*np.array([.10,.40]).T}}
    # Toy examples
    if mapVersion == 0:
        roads = {'1':{'start_int':'d','end_int':'a','oneway':1},\
                 '2':{'start_int':'a','end_int':'b','oneway':1},\
                 '3':{'start_int':'b','end_int':'c','oneway':1},\
                 '4':{'start_int':'c','end_int':'d','oneway':1},\
                 '5':{'start_int':'g','end_int':'b','oneway':1},\
                 '6':{'start_int':'b','end_int':'e','oneway':1},\
                 '7':{'start_int':'e','end_int':'f','oneway':1},\
                 '8':{'start_int':'f','end_int':'g','oneway':1}}
    elif mapVersion == 1:
        roads = {'1':{'start_int':'d','end_int':'a','oneway':0},\
                 '2':{'start_int':'a','end_int':'b','oneway':0},\
                 '3':{'start_int':'b','end_int':'c','oneway':0},\
                 '4':{'start_int':'d','end_int':'c','oneway':0},\
                 '5':{'start_int':'g','end_int':'b','oneway':1},\
                 '6':{'start_int':'b','end_int':'e','oneway':1},\
                 '7':{'start_int':'e','end_int':'f','oneway':1},\
                 '8':{'start_int':'f','end_int':'g','oneway':1},\
                 '9':{'start_int':'a','end_int':'h','oneway':1},\
                 '10':{'start_int':'h','end_int':'g','oneway':1}}
    elif mapVersion == 2 or mapVersion == 3:
        A = intersections['a']['position']
        B = intersections['g']['position']
        r = 0.5*np.sqrt(np.sum(np.power(A - B,2.0)))
        C = 0.5*(B + A)
#        startTheta = np.arctan2(A[1] - C[1],A[0] - C[0])
#        if startTheta < 0:
#            startTheta += 2.0*np.pi
#        endTheta = np.arctan2(B[1] - C[1],B[0] - C[0])
#        if endTheta < 0:
#            endTheta += 2.0*np.pi
#
#        startDir = np.sign(endTheta - startTheta)*np.array([-np.sin(startTheta),np.cos(startTheta)])
#        endDir = np.sign(endTheta - startTheta)*np.array([-np.sin(endTheta),np.cos(endTheta)])
        roads = {'1':{'start_int':'d','end_int':'a','oneway':0},\
                 '2':{'start_int':'a','end_int':'b','oneway':0},\
                 '3':{'start_int':'b','end_int':'c','oneway':0},\
                 '4':{'start_int':'d','end_int':'c','oneway':0},\
                 '5':{'start_int':'g','end_int':'b','oneway':1},\
                 '6':{'start_int':'b','end_int':'e','oneway':1},\
                 '7':{'start_int':'e','end_int':'f','oneway':1},\
                 '8':{'start_int':'f','end_int':'g','oneway':1},\
                 '9c':{'start_int':'a','end_int':'g','oneway':0,\
                       'type':'arc', 'radius':r, 'center':C, 'turn_direction':-1}}
    if mapVersion == 3:
        compute_derived_quantities(roads,intersections)
        (roads,intersections) = add_turn_segments(roads,intersections)
    mg = MapGraph(roads,intersections)
    mg.graph['position_offset'] = np.zeros((2,1))
    mg.graph['mercator_scale'] = 1.0

    return mg

