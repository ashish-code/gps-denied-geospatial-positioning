#!/usr/bin/env python

from itertools import count, izip, chain
import time
import random
import os, sys, subprocess
import csv
import numpy as np
import cPickle as pickle
from copy import copy,deepcopy
from optparse import OptionParser
from ConfigParser import SafeConfigParser
import scipy as sp

from Utils import logsumexp,mkdir_p
from MapGraph import MapGMMDistribution,MapState2RigidTurnDynamics,SINK_ROAD
from MapBuilder import build_toy_map,build_osm_map,mercatorProj

#sp.optimize.minpack

plotDPI = 100
plotFigSize = (5,3.25)

displayDPI = 300
displayCroppedFigSz = (3,3)
displayFigSz = (5,5)

sequenceFrames = {'00':[10,100,110,230], '01':[10,500,880,920], '02':[10,190,210,370], '03':[10,160,240,400], 
                  '05':[10,170,270,460], '06':[150,350,870], '07':[10,100,200,310], '08':[10,100,650,850],
                  '09':[10,100,150,230], '10':[10,70,120,250], 
                  '11':[200,300,400,500], '12':[100,200,300,500], '13':[100,300,450,600], '14':[100,300,500,600],
                  '15':[100,300,450,700], '16':[200,400,500,600], '17':[100,200,400,450], '18':[100,200,400,600],
                  '19':[100,200,300,400], '20':[100,300,500,700], '21':[100,200,350,400]}

topErrNs = range(1,201)
displayErrNs = [1, 5, 10, 20]
plotErrNs = range(1,51)


# Dynamics/Observation parameters
rigidTurn2Params = { 'v_sigma': 0.002, 'dtheta_sigma':0.01, \
                     'dv_sigma': 0.0005, 'gamma':0.1, 'ddtheta_sigma':np.pi/8.0 }

#def partition_graph_greedy(mapgraph,seeds):
#    streetsPerNode = np.ceil(float(len(mapgraph))/size)
#
#    cnode = 0
#    streetsNodeMap = dict()
#    nodeStreetsMap = dict()
#    numCurrStreets = 0
#    
#    unassignedStreets = set(mapgraph.nodes())
#    for (cnode,vt) in enumerate(seeds):
#        streetsNodeMap[vt] = cnode
#        nodeStreetsMap[cnode] = set()
#        nodeStreetsMap[cnode].add(vt)
#        del unassignedStreets[vt]
#        
#    while len(unAssignedStreets) > 0:
#        for cnode in range(len(seeds)):
#            
#        if numCurrStreets == streetsPerNode:
#            cnode += 1
#            numCurrStreets = 0
#            if cnode < size:
#                nodeStreetsMap[cnode] = set()
#    
#        streetsNodeMap[vt] = cnode
#        nodeStreetsMap[cnode].add(vt)
#        numCurrStreets += 1
#    
#    return streetsNodeMap,nodeStreetsMap

def road_midpoint(x,d):
    if x == SINK_ROAD:
        return np.array([0,0])
    else:
        return d['origin'] + 0.5*d['length']*d['direction']

def partition_graph_by_sort(mapgraph,size):
    """ This performs a slightly less stupid partitioning based on a sort of
    road positions.  Road positions are projected onto the direction of maximal
    variance and sorted by their position on that direction. """
    roadPts = np.array([road_midpoint(x,d) for (x,d) in mapgraph.nodes_iter(data = True)]).T
    roadPts -= np.mean(roadPts,1).reshape((2,1))
    roadPtCovar = np.dot(roadPts/np.sqrt(len(mapgraph)-1),roadPts.T/np.sqrt(len(mapgraph)-1))
    (w,V) = np.linalg.eigh(roadPtCovar)
    maxEigI = np.argmax(w)
    roadList = mapgraph.nodes()
    roadList.remove(SINK_ROAD)
    roadList.sort(key = lambda street: np.dot(V[:,maxEigI],mapgraph.node[street]['origin'])) 

    streetsPerNode = np.ceil(float(len(mapgraph))/size)
    streetLenPerNode = mapgraph.totalLength/size
    streetsNodeMap = dict()
    nodeStreetsMap = dict((n,set()) for n in range(size))
    cnode = 0
    numCurrStreets = 0
    currStreetLen = 0
    for vt in roadList:
        if vt == SINK_ROAD:
            continue
        if numCurrStreets == streetsPerNode:
        #if currStreetLen >= streetLenPerNode and cnode+1 < size:
            cnode += 1
            numCurrStreets = 0
            currStreetLen = streetLenPerNode - currStreetLen
    
        streetsNodeMap[vt] = cnode
        if cnode not in nodeStreetsMap:
            nodeStreetsMap[cnode] = set()
        nodeStreetsMap[cnode].add(vt)
        numCurrStreets += 1
        currStreetLen += mapgraph.node[vt]['length']
    
    return streetsNodeMap,nodeStreetsMap
    

def partition_graph_simple(mapgraph,size):
    """ This performs a really stupid partitioning. """
    streetsPerNode = np.ceil(float(len(mapgraph))/size)
    cnode = 0
    streetsNodeMap = dict()
    nodeStreetsMap = dict()
    numCurrStreets = 0
    for vt in mapgraph.nodes():
        if vt == SINK_ROAD:
            continue
        if numCurrStreets == streetsPerNode:
            cnode += 1
            numCurrStreets = 0
    
        streetsNodeMap[vt] = cnode
        if cnode not in nodeStreetsMap:
            nodeStreetsMap[cnode] = set()
        nodeStreetsMap[cnode].add(vt)
        numCurrStreets += 1
    
    return streetsNodeMap,nodeStreetsMap

def compute_node_nbhds(mapgraph,streetsNodeMap,nodeStreetsMap):
    nodeNbhdStreets = dict((cnode,dict()) for cnode in nodeStreetsMap.iterkeys())
    for (cnode,cnodeStreets) in nodeStreetsMap.iteritems():
        for cstreet in cnodeStreets:
            for nstreet in mapgraph.successors(cstreet) + mapgraph.predecessors(cstreet):
                if nstreet == SINK_ROAD:
                    continue
                if streetsNodeMap[nstreet] != cnode:
                    nnode = streetsNodeMap[nstreet]
                    if nnode not in nodeNbhdStreets[cnode]:
                        nodeNbhdStreets[cnode][nnode] = set() 
                    nodeNbhdStreets[cnode][nnode].add(cstreet)
    return nodeNbhdStreets

def partition_graph(mapgraph,size):
    streetsNodeMap,nodeStreetsMap = partition_graph_by_sort(mapgraph,size)
    #streetsNodeMap,nodeStreetsMap = partition_graph_simple(mapgraph,size)
    nodeNbhdStreets = compute_node_nbhds(mapgraph,streetsNodeMap,nodeStreetsMap)
    
    return streetsNodeMap,nodeStreetsMap,nodeNbhdStreets

def determine_local_communications(mapgraph,rank,streetsNodeMap,nodeStreetsMap):
    """ Given the graph partitioning, determine what communications need to happen locally """
    localStreets = nodeStreetsMap[rank]
    localRelevantStreets = copy(localStreets) # local streets plus those incoming to local streets

    outBoundLoad = 0
    inBoundLoad = 0
    
    # Find inbound/outbound nodes/streets
    outNodeStreets = dict() # for each node, what streets do we need to send
    for oe in mapgraph.out_edges_iter(localStreets,data = False):
        if oe[0] == SINK_ROAD or oe[1] == SINK_ROAD:
            continue
        oe_street = oe[1]
        oe_street_node = streetsNodeMap[oe_street]
        if oe_street_node == rank:
            # Check if oe[1] is our own node.
            continue
        if oe_street_node not in outNodeStreets:
            outNodeStreets[oe_street_node] = set()
        outBoundLoad += 1
        outNodeStreets[oe_street_node].add(oe[0])

    for key in outNodeStreets.iterkeys():
        outNodeStreets[key] = sorted(outNodeStreets[key])
    
    inNodeStreets = dict() # for each node, what streets do we need to recv
    for ie in mapgraph.in_edges_iter(localStreets,data = False):
        if ie[0] == SINK_ROAD or ie[1] == SINK_ROAD:
            continue
        ie_street = ie[0]
        ie_street_node = streetsNodeMap[ie_street]
        if ie_street_node == rank:
            # Check if ie[0] is our own node.
            continue
        localRelevantStreets.add(ie_street)
        if ie_street_node not in inNodeStreets:
            inNodeStreets[ie_street_node] = set()
        inBoundLoad += 1
        inNodeStreets[ie_street_node].add(ie[0])

    for key in inNodeStreets.iterkeys():
        inNodeStreets[key] = sorted(inNodeStreets[key])
    
    localComms = [ (rank,x) for x in outNodeStreets.keys() ]
    localComms.extend([ (x,rank) for x in inNodeStreets.keys() ])

    # Sort the local communications to prevent deadlocks
    localComms.sort()

    return localStreets,localRelevantStreets,inNodeStreets,outNodeStreets,localComms,inBoundLoad,outBoundLoad

def update_graph_partition(mapgraph,rank,size,all_dts,streetsNodeMap,nodeStreetsMap,nodeNbhdStreets):
    noisy = False
    origStreetsNodeMap = copy(streetsNodeMap)

    dts = copy(all_dts)
    mean_dt = np.mean(dts)
    std_dt = np.std(dts)
    maxChanges = np.inf
    tryOrder = range(size)
    
    changes = dict()
    nChanges = 0
    # Determine which streets to shift around
#    while std_dt/mean_dt > 0.25 and nChanges < maxChanges:
#    while np.max(np.abs(dts-mean_dt))/mean_dt > 0.1 and nChanges < maxChanges:
    while (np.max(dts) - np.min(dts))/np.sum(dts) > 0.05 and nChanges < maxChanges:
        tryOrder.sort(key=lambda x: np.abs(dts[x] - mean_dt),reverse=True)
#        tryOrder.sort(key=lambda x: dts[x],reverse=True)

        for updateRank in tryOrder:
            if dts[updateRank] < mean_dt:
                if noisy and rank == 0:
                    print 'Attempting to add a street to {0}'.format(updateRank)
    
                toRank = updateRank
                fromRank = None
                for orank in nodeNbhdStreets.iterkeys():
                    if toRank not in nodeNbhdStreets[orank]:
                        continue
                    if dts[orank] > dts[toRank] and (fromRank == None or dts[orank] > dts[fromRank]) and \
                       len(nodeNbhdStreets[orank][toRank]) > 0 and (toRank,orank) not in changes:
                        fromRank = orank
            else:
                if noisy and rank == 0:
                    print 'Attempting to remove a street from {0}'.format(updateRank)
    
                fromRank = updateRank
                toRank = None
                for orank in nodeNbhdStreets[updateRank].iterkeys():
                    if dts[orank] < dts[fromRank] and (toRank == None or dts[orank] < dts[toRank]) and \
                       len(nodeNbhdStreets[fromRank][orank]) > 0 and (orank,fromRank) not in changes:
                        toRank = orank
    
            if toRank == None or fromRank == None:
                if noisy and rank == 0:
                    print 'No suitable neighbours of {0}'.format(updateRank)
                continue
            else:
                if noisy and rank == 0:
                    print 'Attempting to move a street {1} -> {0}'.format(toRank, fromRank)

            assert(len(nodeNbhdStreets[fromRank][toRank]) > 0)
            # Try to take a street from node fromRank and give it to node toRank
            stealStreet = None
            for k in nodeNbhdStreets[fromRank][toRank]:
                if origStreetsNodeMap[k] == fromRank:
                    stealStreet = k
                    break

            if stealStreet == None:
                # Nothing on the border between these two that hasn't already been stolen
                if noisy and rank == 0:
                    print 'No bordering streets between {0} and {1}'.format(fromRank,toRank)
                continue
            else:
                break
            
        if toRank == None or fromRank == None or stealStreet == None:
            # Nothing to steal
            if noisy and rank == 0:
                print 'No suitable moves available'
            break

        if noisy and rank == 0:
            print '  Moving {0} from {1} to {2}'.format(stealStreet,fromRank,toRank)

        # Update the node neighbourhood information.
        nodeNbhdStreets[fromRank][toRank].remove(stealStreet)
        for orank in nodeNbhdStreets[fromRank].iterkeys():
            nodeNbhdStreets[fromRank][orank].discard(stealStreet)
        nodeStreetsMap[fromRank].remove(stealStreet)
        nodeStreetsMap[toRank].add(stealStreet)
        streetsNodeMap[stealStreet] = toRank
        
        for nstreet in chain(mapgraph.successors_iter(stealStreet),mapgraph.predecessors_iter(stealStreet)):
            if nstreet == SINK_ROAD:
                continue
            if streetsNodeMap[nstreet] != toRank:
                nnode = streetsNodeMap[nstreet]
                if nnode not in nodeNbhdStreets[toRank]:
                    nodeNbhdStreets[toRank][nnode] = set() 
                nodeNbhdStreets[toRank][nnode].add(stealStreet)

        # Make a note to communicate the information
        if (fromRank,toRank) not in changes:
            changes[(fromRank,toRank)] = []
        changes[(fromRank,toRank)].append((stealStreet,mapgraph.predecessors(stealStreet)))
            
        timeUpd = dts[fromRank]/(len(nodeStreetsMap[fromRank]) + 1)

        prevdts = dts
        dts[toRank] += timeUpd
        dts[fromRank] -= timeUpd

        mean_dt = np.mean(dts)
        std_dt = np.std(dts)
        nChanges += 1
    
    return changes

def do_mpi_update(options,updateData,comm,rank,size,localComms,inNodeStreets,outNodeStreets,localStreets,localRelevantStreets,mapgraph):
    logDiscardCompThresh = np.log(1.0e-10/len(mapgraph)) # Threshold at which a component is discarded
    post = updateData[0]
                
    # Wait for everyone to catch up
    comm.Barrier()

    curr_startt = time.time()
    # Send/Recv logsum data to/from everyone else
    partialLogSumVs = np.array(comm.allgather(updateData[1]))
    logsumV = logsumexp(np.array(partialLogSumVs))
        
    # Perform local communications of posterior information
    for currComm in localComms:
        src = currComm[0]
        dst = currComm[1]
        if src == rank:
            # We're sending to someone
            sendNodes = outNodeStreets[dst]
            sendPost = post.extractData(sendNodes)
            comm.send(sendPost, dest=dst, tag=src)
        elif dst == rank:
            # We're recving from someone
            recvNodes = inNodeStreets[src]
            recvPost = comm.recv(source=src, tag=src)
            post.updateData(recvNodes,recvPost)
        
    # Normalize the posterior now that we have all the needed information
    for vt in localRelevantStreets:
        post.logV[vt] -= logsumV
        if np.isfinite(post.logV[vt]) and post.logV[vt] <= logDiscardCompThresh:
            post.logV[vt] = -np.inf
            post.Theta[vt] = ()

    if rank == 0:
        all_logV = comm.gather(None)
    else:
        all_logV = comm.gather(dict((k, post.logV[k]) for k in localStreets))
    if rank == 0:
        for (orank,clogV) in enumerate(all_logV[1:]):
            post.logV.update(clogV)
    comm_dt = time.time() - curr_startt

    return (post, logsumV, comm_dt)

def get_map_name(mapspec):
    if mapspec.endswith('.bin') or  mapspec.endswith('.p'):
        mapName = mapspec.rpartition('.')[0]
    elif '.bin:' in mapspec:
        mapParts = mapspec.partition(':')
        mapfile = mapParts[0]
        rangeVals = mapParts[2].split(',')
        lat_range = (float(rangeVals[0]),float(rangeVals[1]))
        lon_range = (float(rangeVals[2]),float(rangeVals[3]))
        mapName = '{0}_sublat{1}-{2}_sublon{3}-{4}'.format(mapfile.partition('.')[0],lat_range[0],lat_range[1],lon_range[0],lon_range[1])
    else:
        mapName = mapspec
        
    return mapName

def load_mapgraph(options):
    mapspec = options.map
    doSimplify = options.simplify_map
    if mapspec.endswith('.bin'):
        mapgraph = build_osm_map(mapspec,doSimplify)
        mapVersion = None
    elif mapspec.endswith('.p'):
        mapgraph = pickle.load(open(mapspec,'rb'))
        mapVersion = None
    elif '.bin:' in mapspec:
        mapParts = mapspec.partition(':')
        mapfile = mapParts[0]
        rangeVals = mapParts[2].split(',')
        lat_range = (float(rangeVals[0]),float(rangeVals[1]))
        lon_range = (float(rangeVals[2]),float(rangeVals[3]))
        mapgraph = build_osm_map(mapfile,doSimplify,lat_range = lat_range,lon_range = lon_range)
        mapVersion = None
    else:
        mapVersion = int(mapspec)
        mapgraph = build_toy_map(mapVersion)
    return mapgraph,mapVersion

def parse_txtobs_line(line):
    Rt = np.fromstring(line,sep = ' ');
    return np.vstack([Rt.reshape(3,4),np.array([0, 0, 0, 1])])

def load_txtobs(inputfilename,dt):
    obs = []
    t = 0
    Rt = []
    odomDataPos = np.zeros(2)
    odomDataAng = 0
    fileObs = open(inputfilename,'rt').readlines()
    for (tInd,line) in enumerate(fileObs):
        prevRt = Rt
        Rt = parse_txtobs_line(line)
        if tInd == 0:
            continue
        localRt = np.linalg.solve(prevRt,Rt)
        forwardDist = localRt[2,3]
        angChange = np.arcsin(localRt[0,2])
        
        obsData = np.array([forwardDist/1000.0/dt,-angChange/dt]).reshape(2,1)
#        odomDataPos = Rt[(0,2),3]/1000.0
        cdir = np.array([np.sin(odomDataAng), np.cos(odomDataAng)])
        odomDataPos += dt*obsData[0,0]*cdir
        odomDataAng += angChange

        t += dt
        cobs = {'t':t,'tInd':tInd-1,'dt':dt,'obs':obsData,'odom_int_pos':copy(odomDataPos),'odom_int_ang':odomDataAng}
        obs.append(cobs)

    return obs

def load_obs(inputfilename,dt):
    obs = []
    t = 0
    int_ang = 0
    int_pos = np.zeros(2)
    fileObs = open(inputfilename,'rt').readlines()
    for (tInd,line) in enumerate(fileObs):
        splitLine = line.split()
        odomobs = np.array([float(splitLine[0])/1000.0, float(splitLine[1])]).reshape(2,1)/dt
#        int_pos = np.array([float(splitLine[2]), float(splitLine[3])]).reshape(2)
        cdir = np.array([np.sin(int_ang), np.cos(int_ang)])
        int_pos += dt*odomobs[0,0]*cdir
        int_ang += -float(splitLine[1])
        t += dt
        cobs = {'t':t,'tInd':tInd,'dt':dt,'obs':odomobs, 'odom_int_pos':copy(int_pos),'odom_int_ang':int_ang}
        obs.append(cobs)

    return obs

def transform_obs(obs,dist_thresh,ang_thresh,dt_thresh,data_skip):
    newObs = []
    dist = 0
    ang = 0
    prevt = 0
    skippedTInd = 0
    for cOldObs in obs:
        dist += cOldObs['obs'][0]*cOldObs['dt']
        ang += cOldObs['obs'][1]*cOldObs['dt']
        t = cOldObs['t']
        skippedTInd += 1
        if dist > dist_thresh or np.abs(ang) > ang_thresh or t - prevt > dt_thresh or skippedTInd%data_skip == 0:
            cobs = {'t':t,'tInd':cOldObs['tInd'],'dt':t-prevt,'obs':np.array([dist/(t-prevt),ang/(t-prevt)]).reshape(2,1)}
            if 'odom_int_pos' in cOldObs:
                cobs['odom_int_pos'] = cOldObs['odom_int_pos']
                cobs['odom_int_ang'] = cOldObs['odom_int_ang']
            newObs.append(cobs)
            dist = 0
            ang = 0
            prevt = t
            skippedTInd = 0
            
    return newObs

def load_observations(options):
    return load_observations_file(options.odometry,options.dt,options.data_skip)

def load_observations_file(odometry,dt,data_skip):
    if odometry.endswith('.obs'):
        origobs = load_obs(odometry,dt)
    elif odometry.endswith('.txt'):
        origobs = load_txtobs(odometry,dt)
    elif odometry.endswith('.p'):
        origobs = pickle.load(open(odometry,'rb'))
    else:
        assert(False)

    if data_skip != None and data_skip > 1:
        dist_thresh = np.inf
        ang_thresh = np.inf
        dt_thresh = np.inf 
        obs = transform_obs(origobs,dist_thresh,ang_thresh,dt_thresh,data_skip)
    else:
        obs = origobs
        
    return obs

def load_mapdynamics(options):
    if options.dynamics == None or options.dynamics == 'state2rigid':
        if options.dynamics_params != None and options.dynamics_params != '':
            dynOpts = pickle.load(open(options.dynamics_params,'rb'))
        else:
            dynOpts = rigidTurn2Params

        dynOpts['dt'] = options.dt
        dynOpts['dt_1'] = options.dt
        mapdynamics = MapState2RigidTurnDynamics(dynOpts)
    elif options.dynamics == 'state2rigid_1kind':
        if options.dynamics_params != None and options.dynamics_params != '':
            dynOpts = pickle.load(open(options.dynamics_params,'rb'))
        else:
            dynOpts = rigidTurn2Params

        dynOpts['dt'] = options.dt
        dynOpts['dt_1'] = options.dt
        mapdynamics = MapState2RigidTurnDynamics(dynOpts,kinds=False)
    else:
        assert False,'Unknown dynamics type: {0}'.format(options.dynamics)

    return mapdynamics

def load_crop_range(options,mapgraph):
    if options.crop_size == None:
        return None
    
    assert options.gps_data != None, 'Ground truth GPS data required for cropping'

    stateIter = iter(open(options.gps_data,'rt'))
    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']
    latlon = np.empty(2)

    statet = stateIter.next()
    lineParts = statet.split()
    latlon[0] = float(lineParts[0])
    latlon[1] = float(lineParts[1])
    gtPos = mercatorProj(latlon,projScale) - ptCoordOrigin
    
    return (gtPos - 0.5*options.crop_size,gtPos + 0.5*options.crop_size)

def do_inference(options,args):
    assert(len(args) == 1)

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0
 
    if rank == 0:
        print '\nRunning inference with config file {0}'.format(args[0])

    load_config_file(options,args[0])

    if options.start_frame == None:
        start_frame = 0
    else:
        start_frame = options.start_frame        

    debugNodes = set()
    #debugNodes = set([('af052d3b365e654802f9d1e30e65073a63f81244', 1), ('cacad72e88e2b2de953ab0f61be79b255ef3521e', 1), ('e8fd92770e7641ebdae3b8f35bd2304183e25695.0003', -1)])
    tmpt = time.time()
    if rank == 0: print 'Loading mapgraph...',
    mapgraph,mapVersion = load_mapgraph(options)
    if rank == 0: print 'done in {0}s.'.format(time.time() - tmpt)
    cropRange = load_crop_range(options,mapgraph)

    if options.mpi:
        # Partition the street nodes amongst the workers
        tmpt = time.time()
        if rank == 0: print 'Partitioning street graph...',
        streetsNodeMap,nodeStreetsMap,nodeNbhdStreets = partition_graph(mapgraph,size)
        if rank == 0: print 'done in {0}s.'.format(time.time() - tmpt)
        
        # Figure out the communication pattern for the local node
        tmpt = time.time()
        if rank == 0: print 'Determining communication patterns...',
        localStreets,localRelevantStreets, \
        inNodeStreets,outNodeStreets, \
        localComms,inBoundLoad,outBoundLoad = \
                     determine_local_communications(mapgraph,rank,streetsNodeMap,nodeStreetsMap)
        if rank == 0: print 'done in {0}s.'.format(time.time() - tmpt)
        comm.Barrier()
    else:
        localStreets = None
        localRelevantStreets = None
        
    if rank == 0:
        sortedStreets = mapgraph.nodes()
        sortedStreets.sort()

    if options.syn_path == None:
        path = None
        pathLength = None
    elif options.syn_path.startswith('rand'):
        path = None
        pathLength = int(options.syn_path.partition(':')[2])
    elif options.syn_path == 'default':
        if mapVersion < 2:
            path = (('1',1),('2',1),('6',1),('7',1),('8',1),('5',1),('3',1),('4',-1),('1',1),('2',1),('3',1))
        else:
            path = (('1',1),('9c',1),('5',1),('3',1),('4',-1),('1',1),('2',1),('3',1))
        pathLength = None
    else:
        path = options.syn_path.split(':')
        pathLength = None
        
    if rank == 0:
        # In the root node load the data and setup the map
        # Build the map and map dynamics
        mapdynamics = load_mapdynamics(options)
        
        if options.odometry == None:
            states = mapdynamics.synthesize_path(mapgraph,options.syn_speed,path,pathLength,True)
            obs = mapdynamics.synthesize_odometry(mapgraph,states)
        else:
            states = None
            obs = load_observations(options)
        
        data = {'mapdynamics':mapdynamics,'states':states,'obs':obs,'size':size,'mpi':options.mpi, \
                'start_frame':start_frame,'crop_range':cropRange}
        pickle.dump(data, open(get_datafile_path(options),'wb'))
        stats_pkl_file = open(get_outfile_path(options,'stats.p'),'wb')
    else:
        data = None
    
    if options.mpi:
        # Broadcast the data to all nodes
        data = comm.bcast(data, root = 0)
        obs = data['obs']
        mapdynamics = data['mapdynamics']
        del data

    np.seterr(divide='ignore')
    np.random.seed()
    
    # Initialize the posterior
    post = MapGMMDistribution()
    post.setUniformDist(mapgraph, mapdynamics, localRelevantStreets, options.speed, (0.5*options.speed)**2, cropRange=cropRange)
    if cropRange != None:
        # If we've cropped the initialization, then we need to normalize the posterior 
        if options.mpi:
            partialLogSumVs = np.array(comm.allgather(post.computeLogVSum(mapgraph,localStreets)))
            logsumV = logsumexp(partialLogSumVs)
            for v in localRelevantStreets:
                post.logV[v] -= logsumV
        else:
            logsumV = post.computeLogVSum(mapgraph)
            print 'logV normalization: {0}'.format(logsumV)
            for v in mapgraph.nodes_iter():
                if v == SINK_ROAD:
                    continue
                post.logV[v] -= logsumV

    # Open the file for dumping the posteriors as we go
    if options.mpi:
        post_pkl_file = open(get_outfile_path(options,'posteriors-rank{0:03}.p'.format(rank)),'wb')

        # Wait for everyone to catch up
        comm.Barrier()
        
        movingAvg_dts = np.zeros((size,3))
        sum_dts = np.zeros(size)
        sum_dts_N = 0
    else:
        post_pkl_file = open(get_outfile_path(options,'posteriors.p'),'wb')

    pickle.dump(post,post_pkl_file)
    
    sum_wall_dt = 0
    sum_iter_dt = 0
    num_iter = 0
    max_wall_dt = 0
    min_wall_dt = np.inf
    prev_data_dt = obs[0]['dt']
    global_startt = time.time()
    for curr_obs in obs:
        tInd = curr_obs['tInd']
        t = curr_obs['t']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']

        mapdynamics.setParameters({'dt':curr_data_dt,'dt_1':prev_data_dt})
        if tInd < start_frame:
            continue
        iter_startt = time.time()

        if rank == 0:
            print 'tInd = {0}, t = {1}, yt = {2} '.format(tInd,t,yt.T),
            
        # Perform local update
        filter_startt = time.time()
        updateData = post.filterDist(mapgraph,mapdynamics,yt,localKeys = localStreets,filterVer = options.filter_version,simplifyThresh = options.simplify_posterior*options.simplify_threshold)
        filter_dt = time.time() - filter_startt
        if options.mpi:
            (post,logEvidence,comm_dt) = do_mpi_update(options,updateData,comm,rank,size,localComms,inNodeStreets,outNodeStreets,localStreets,localRelevantStreets,mapgraph)

            dump_startt = time.time()
            frameRef = post_pkl_file.tell()
            pickle.dump(post.extract(localStreets),post_pkl_file)
            dump_dt = time.time() - dump_startt
            
            curr_dts = (filter_dt,comm_dt,dump_dt)

            # Wait for everyone to catch up
            comm.Barrier()
            if rank == 0:
                frameRefs = comm.gather(frameRef)
            else:
                frameRefs = comm.gather(frameRef)
            
            all_dts = np.array(comm.allgather(curr_dts))
            movingAvg_dts = 0.5*(all_dts + movingAvg_dts)
            sum_dts += np.sum(all_dts,1)
            sum_dts_N += 1
            avg_dts = sum_dts/sum_dts_N

            if options.load_balance_freq > 0 and sum_dts_N >= options.load_balance_freq:
                partitionChanges = update_graph_partition(mapgraph,rank,size,avg_dts,streetsNodeMap,nodeStreetsMap,nodeNbhdStreets)
                comms = []
            else:
                partitionChanges = ()
                comms = []

            if len(partitionChanges) > 0:
                localStreets,localRelevantStreets, \
                inNodeStreets,outNodeStreets, \
                localComms,inBoundLoad,outBoundLoad = \
                             determine_local_communications(mapgraph,rank,streetsNodeMap,nodeStreetsMap)

                # Transfer relevant posterior information
                comms = partitionChanges.keys()
                comms.sort()
                changed = False
                for (fromRank,toRank) in comms:
                    if rank == fromRank or rank == toRank:
                        changeList = partitionChanges[(fromRank,toRank)]
                        changeSet = set()
                        for (bStreet,streetPreds) in changeList:
                            changeSet.add(bStreet)
                            for tStreet in streetPreds:
                                changeSet.add(tStreet)
                        changeList = sorted(changeSet)

                    if rank == fromRank:
                        # We're sending to someone
                        sendPost = post.extractData(changeList)
                        comm.send(sendPost, dest=toRank, tag=fromRank)
                        changed = True
                    elif rank == toRank:
                        # We're recving from someone
                        recvPost = comm.recv(source=fromRank, tag=fromRank)
                        post.updateData(changeList,recvPost)
                        changed = True

                if changed:
                    # If we lost or gained a street, discard the extra info
                    if rank == 0:
                        # FIXME: Do this better
                        origLogV = post.logV
                        post = post.extract(localRelevantStreets)
                        post.logV = origLogV
                    else:
                        post = post.extract(localRelevantStreets)

                sum_dts = np.zeros(size)
                sum_dts_N = 0
        else:
            post = updateData[0]
            logEvidence = updateData[1]
            frameRefs = [post_pkl_file.tell()]
            pickle.dump(post,post_pkl_file)


        if rank == 0:
            if not options.disable_postprint:
                for vt in sortedStreets:
                    if vt != SINK_ROAD and np.isfinite(post.logV[vt]):
                        print 'p[{0}] = {1}'.format(vt, np.exp(post.logV[vt]))

            currt = time.time()
            wall_dt = currt - iter_startt
            max_wall_dt = np.maximum(wall_dt,max_wall_dt)
            min_wall_dt = np.minimum(wall_dt,min_wall_dt)
            if options.mpi:
                iter_dt = np.sum(all_dts)
#                print 'Load %: {0}, Max imbalance {1}%'.format(100.0*avg_dts/np.sum(avg_dts),100.0*(np.max(avg_dts) - np.min(avg_dts))/np.sum(avg_dts))
#                for (fromRank,toRank) in comms:
#                    moves = [street for (street,predStreets) in partitionChanges[(fromRank,toRank)]]
#                    print '{0} -> {1}: {2}'.format(fromRank,toRank,moves)
                    

##                print all_dts
#                summvdts = np.sum(movingAvg_dts,1)
#                print np.sum(movingAvg_dts,1)
#                print np.max(np.abs(summvdts - np.mean(summvdts)))/np.mean(summvdts)
#                print np.mean(movingAvg_dts,0)
#                print np.std(movingAvg_dts,0)
#                if sum_dts_N == 0:
#                    for orank in range(size):
#                        print 'Node {0}: {1} streets'.format(orank,len(nodeStreetsMap[orank]))
                pickle.dump({'logEvidence':logEvidence,'iter_dt':iter_dt,'wall_dt':wall_dt,'all_dts':all_dts,'avg_dts':avg_dts,'post_pos_ref':frameRefs},stats_pkl_file)
            else:
                iter_dt = wall_dt 
                pickle.dump({'logEvidence':logEvidence,'iter_dt':iter_dt,'wall_dt':wall_dt,'post_pos_ref':frameRefs},stats_pkl_file)
            sum_wall_dt += wall_dt
            sum_iter_dt += iter_dt
            num_iter += 1
            #print 'Elapsed time {0}s\n   wall_dt {1}s (min {2}s, max {3}s, avg {4}s)\n   wall_dt/tInd {5}s, iter_dt {6}s (avg {7}s)'.format(currt-global_startt,wall_dt,min_wall_dt,max_wall_dt,sum_wall_dt/num_iter,float(currt-global_startt)/(tInd+1),iter_dt,sum_iter_dt/num_iter)
            print 'Elapsed time {0}, wall_dt/tInd {5}s'.format(currt-global_startt,wall_dt,min_wall_dt,max_wall_dt,sum_wall_dt/num_iter,float(currt-global_startt)/(tInd+1),iter_dt,sum_iter_dt/num_iter)
#            print 'Elapsed time {0}s, time on last obs {2}s\n  s/obs: avg = {1}s, min = {3}s, max = {4}s'.format(currt-global_startt,float(currt-global_startt)/(tInd+1),iter_dt,min_wall_dt,max_wall_dt)

        post_pkl_file.flush()
        if rank == 0:
            stats_pkl_file.flush()
        prev_data_dt = curr_data_dt

    post_pkl_file.close()
    if rank == 0:
        stats_pkl_file.close()

def do_print_timing_stats(options,args):
    load_config_file(options,args)

    # Get data file and statistics
    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    mapdynamics = baseData['mapdynamics']
    start_frame = baseData['start_frame']
    stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')
    
    all_dts_sum = 0
    wall_dt_sum = 0
    ndts = 0

    for curr_obs in baseData['obs']:
        tInd = curr_obs['tInd']
        if tInd < start_frame:
            continue
        
        t = curr_obs['t']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']
        stepData = pickle.load(stats_pkl_file)
        all_dts = np.array(stepData['all_dts'])
        wall_dt =  stepData['wall_dt']
        
        print 'tInd = {0}, t = {2}, obst = {1}'.format(tInd,yt.T,t)
        print '  filter = {0}s'.format(np.mean(all_dts[:,0]))
        print '  comm = {0}s'.format(np.mean(all_dts[:,1]))
        print '  dump = {0}s'.format(np.mean(all_dts[:,2]))
        print '  other = {0}s'.format(wall_dt - np.max(np.sum(all_dts,1)))
        
        ndts += 1
        all_dts_sum += all_dts
        wall_dt_sum += wall_dt

    stats_pkl_file.close()
    
    all_dts_mean = all_dts_sum/ndts
    wall_dt_mean = wall_dt_sum/ndts

    print 'Averages:'
    print '  filter = {0}s'.format(np.mean(all_dts_mean[:,0]))
    print '  comm = {0}s'.format(np.mean(all_dts_mean[:,1]))
    print '  dump = {0}s'.format(np.mean(all_dts_mean[:,2]))
    print '  other = {0}s'.format(wall_dt_mean - np.max(np.sum(all_dts_mean,1)))


def adjust_box(boxPos,boxSz,globPos,globSz):
    for i in range(2):
        if (boxPos[i] + 0.5*boxSz[i] > globPos[i] + 0.5*globSz[i]) and (boxPos[i] - 0.5*boxSz[i] < globPos[i] - 0.5*globSz[i]):
            boxPos[i] = globPos[i]

        elif boxPos[i] + 0.5*boxSz[i] > globPos[i] + 0.5*globSz[i]:
            boxPos[i] = (globPos[i] + 0.5*globSz[i]) - 0.5*boxSz[i]

        elif boxPos[i] - 0.5*boxSz[i] < globPos[i] - 0.5*globSz[i]:
            boxPos[i] = (globPos[i] - 0.5*globSz[i]) + 0.5*boxSz[i]

def draw_box(ax,boxPos,boxSz):
    return ax.plot( [ boxPos[0] - 0.5*boxSz[0], boxPos[0] + 0.5*boxSz[0], boxPos[0] + 0.5*boxSz[0], boxPos[0] - 0.5*boxSz[0], boxPos[0] - 0.5*boxSz[0] ], \
                    [ boxPos[1] - 0.5*boxSz[1], boxPos[1] - 0.5*boxSz[1], boxPos[1] + 0.5*boxSz[1], boxPos[1] + 0.5*boxSz[1], boxPos[1] - 0.5*boxSz[1] ], \
                    linewidth = 2.0, color='red', alpha=0.75 )
#             linewidth = 5.0, linestyle='--', color='black', alpha=0.5 )
    
def do_display_frame(orig_options,args):
    kittiFrameFormat = '/Media/Results/KITTI/dataset/sequences/{seqId}/image_0/{frameNum:06d}.png'
    regionSz = 500.0/1000.0
    boxPosLag = 0.85
    boxSzLag = 0.9

    import matplotlib
    if orig_options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    import matplotlib.cm as cm
    import matplotlib.transforms as mptrans

    if orig_options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    frameCount = 0
    fig = plt.figure(dpi = 0.25*displayDPI,figsize=(2.0*(5.0/4.0)*displayFigSz[1],2.0*displayFigSz[1]))
    fig.subplots_adjust(left=0,right=1,top=1,bottom=0,wspace=0,hspace=0)

    for cfgFile in args:
        options = deepcopy(orig_options)
        load_config_file(options,cfgFile)

        mapgraph,mapVersion = load_mapgraph(options)
        fullMapSz = mapgraph.posMax - mapgraph.posMin
        fullMapCenter = 0.5*(mapgraph.posMax + mapgraph.posMin)

        majorTickSz = 500.0/1000.0
        while np.any((fullMapSz/majorTickSz) > 12):
            majorTickSz *= 2.0
        minorTickSz = majorTickSz/5.0

        projScale = mapgraph.graph['mercator_scale']
        ptCoordOrigin = mapgraph.graph['position_offset']
        latlon = np.empty(2)

        sortedStreets = sorted(mapgraph.nodes_iter())

        baseData = pickle.load(open(get_datafile_path(options),'rb'))
        mapdynamics = baseData['mapdynamics']
        start_frame = baseData['start_frame']
        stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')
        modes_pkl_file = open(get_outfile_path(options,'modes.p'),'rb')
        nmsParams = pickle.load(modes_pkl_file)

        # Open the file with the posteriors
        if baseData['mpi']:
            post_pkl_files = [ open(get_outfile_path(options,'posteriors-rank{0:03}.p'.format(crank)),'rb') for crank in range(baseData['size']) ]
            post = None
            for pfile in post_pkl_files:
                if post == None:
                    post = pickle.load(pfile)
                else:
                    post.update(pickle.load(pfile))
        else:
            post_pkl_file = open(get_outfile_path(options,'posteriors.p'),'rb')
            post = pickle.load(post_pkl_file)

        # Open the GPS data
        if options.gps_data != None:
            stateIter = iter(open(options.gps_data,'rt'))

        majorXTicks = np.arange(mapgraph.posMin[0] - 1.0,mapgraph.posMax[0] + 1.0, majorTickSz)
        majorYTicks = np.arange(mapgraph.posMin[1] - 1.0,mapgraph.posMax[1] + 1.0, majorTickSz)
        minorXTicks = np.arange(mapgraph.posMin[0] - 1.0,mapgraph.posMax[0] + 1.0, minorTickSz)
        minorYTicks = np.arange(mapgraph.posMin[1] - 1.0,mapgraph.posMax[1] + 1.0, minorTickSz)

        # Get the image axes
        imgAX = fig.add_subplot(2,1,1)
        leftAX = fig.add_subplot(2,2,3)
        rightAX = fig.add_subplot(2,2,4) 

        # Display the map on the left
        cmin = fullMapCenter - 0.5*np.max(fullMapSz)
        cmax = fullMapCenter + 0.5*np.max(fullMapSz)
        leftAX.set_xlim((cmin[0],cmax[0]))
        leftAX.set_ylim((cmin[1],cmax[1]))
        leftAX.set_xticks(majorXTicks[np.logical_and(majorXTicks >= cmin[0],majorXTicks <= cmax[0])])
        leftAX.set_yticks(majorYTicks[np.logical_and(majorYTicks >= cmin[1],majorYTicks <= cmax[1])])
        leftAX.set_xticks(minorXTicks[np.logical_and(minorXTicks >= cmin[0],minorXTicks <= cmax[0])],minor=True)
        leftAX.set_yticks(minorYTicks[np.logical_and(minorYTicks >= cmin[1],minorYTicks <= cmax[1])],minor=True)
        leftAX.set_xticklabels([]) 
        leftAX.set_yticklabels([])
        leftAX.grid(True,'major',alpha=0.75)
        mapgraph.display(fig,leftAX)
        leftAX.text(0.5, 0.95, 'Region Map', ha='center', va='baseline', 
                    bbox={'facecolor':'white','alpha':0.5,'edgecolor':'none'},
                    fontsize=14, fontweight='bold', transform=leftAX.transAxes)

        dispBoxMin = cmin
        dispBoxMax = cmax
        dispBoxPos = 0.5*(dispBoxMin + dispBoxMax)
        dispBoxSz = dispBoxMax - dispBoxMin

        odomIntPos = []
        if options.gps_data != None:
            prevGTs = []
            prevOdomPts = []
        else:
            prevGTs = None
            prevOdomPts = None
        
        stateInd = -1
        for curr_obs in baseData['obs']:
            t = curr_obs['t']
            tInd = curr_obs['tInd']
            curr_data_dt = curr_obs['dt']
            yt = curr_obs['obs']
            
            if tInd < start_frame:
                continue

            # Read GPS data to get the ground truth position
            if options.gps_data != None:
                while stateInd < tInd:
                    statet = stateIter.next()
                    stateInd += 1
                    lineParts = statet.split()
                    latlon[0] = float(lineParts[0])
                    latlon[1] = float(lineParts[1])
                    heading = np.pi/2.0 - float(lineParts[5])
                    gtPos = mercatorProj(latlon,projScale) - ptCoordOrigin

                    if stateInd == 0:
                        # Use the first point to orient the odometry (currently not used)
                        odomIntRot = np.array([(np.cos(heading), np.sin(heading)), (-np.sin(heading),np.cos(heading))])
                        odomIntOrigin = np.copy(gtPos)
                    currGtX = {'latlon':copy(latlon)}

                    if stateInd >= start_frame:
                        prevGTs.append(deepcopy(currGtX))

                prevOdomPts.append(odomIntOrigin + np.dot(odomIntRot,curr_obs['odom_int_pos']))
            else:
                currGtX = None

            odomIntPos.append(curr_obs['odom_int_pos'])
            currOdomIntRot = np.array([( np.cos(curr_obs['odom_int_ang']), np.sin(curr_obs['odom_int_ang'])), \
                                       (-np.sin(curr_obs['odom_int_ang']), np.cos(curr_obs['odom_int_ang']))])

            topModes = pickle.load(modes_pkl_file)
            stepData = pickle.load(stats_pkl_file)

            # Find the bounding box of the current set of modes
            modesMin = mapgraph.posMax
            modesMax = mapgraph.posMin
            for (p,curr_street,curr_pos,curr_heading) in topModes:
                modesMin = np.minimum(modesMin.reshape(2),curr_pos.reshape(2))
                modesMax = np.maximum(modesMax.reshape(2),curr_pos.reshape(2))

            # Update the display box based on the current set of modes
    #        dispBoxMin = boxLag*dispBoxMin + (1.0-boxLag)*modesMin
    #        dispBoxMax = boxLag*dispBoxMax + (1.0-boxLag)*modesMax
    #        dispBoxPos = 0.5*(dispBoxMin + dispBoxMax)
    #        dispBoxSz = dispBoxMax - dispBoxMin
            dispBoxPos = boxPosLag*dispBoxPos + (1.0-boxPosLag)*0.5*(modesMin + modesMax)
            dispBoxSz = boxSzLag*dispBoxSz + (1.0-boxSzLag)*(modesMax - modesMin)
            dispBoxSz[0:2] = np.maximum(np.max(dispBoxSz),regionSz)
            
            # Adjust it so that it doesn't move outside the bounds of the map region
            adjust_box(dispBoxPos,dispBoxSz,fullMapCenter,fullMapSz)

            dispBoxMin = dispBoxPos - 0.5*dispBoxSz
            dispBoxMax = dispBoxPos + 0.5*dispBoxSz

            # frameCount is used to keep track of which worker is processing which frame
            frameCount += 1
            if tInd < options.resume or not (frameCount%size == rank):
                continue

            # Load the posterior for the current frame
            if baseData['mpi']:
                for (frameRef,pfile) in izip(stepData['post_pos_ref'],post_pkl_files):
                    pfile.seek(frameRef)
                    post.update(pickle.load(pfile))
            else:
                post_pkl_file.seek(stepData['post_pos_ref'][0])
                post = pickle.load(post_pkl_file)
                
            # Load the image data for the current frame
            currImg = mpimg.imread(kittiFrameFormat.format(seqId = options.sequence_id, frameNum = tInd + 1))
            imgAX.cla()
            imgAX.imshow(currImg,cmap=cm.Greys_r)
            imgAX.text(0.5, 0.925, 'Input Image (left camera)', ha='center', va='baseline',
                       bbox={'facecolor':'white','alpha':0.5,'edgecolor':'none'},
                       fontsize=14, fontweight='bold', transform=imgAX.transAxes)
            imgAX.set_xticks([]) 
            imgAX.set_yticks([])
            imgAX.set_aspect('equal')

            # Draw the visual odometry on the image
            odomBoxPos = np.array([0.15,0.5])
            odomBoxSz = np.array([0.25,0.9])
            odomBoxMin = odomBoxPos - 0.5*odomBoxSz
            odomBoxMax = odomBoxPos + 0.5*odomBoxSz
#            odomBoxOrigin = np.array([0.15,0.7])
            odomBoxOrigin = np.array([0.15,0.5])
            odomBoxScale = 1.05*np.array([1.0,1.0/imgAX.get_data_ratio()])
            imgAX.fill( [ odomBoxPos[0] - 0.5*odomBoxSz[0], odomBoxPos[0] + 0.5*odomBoxSz[0], odomBoxPos[0] + 0.5*odomBoxSz[0], odomBoxPos[0] - 0.5*odomBoxSz[0], odomBoxPos[0] - 0.5*odomBoxSz[0] ], \
                        [ odomBoxPos[1] - 0.5*odomBoxSz[1], odomBoxPos[1] - 0.5*odomBoxSz[1], odomBoxPos[1] + 0.5*odomBoxSz[1], odomBoxPos[1] + 0.5*odomBoxSz[1], odomBoxPos[1] - 0.5*odomBoxSz[1] ], \
                        edgecolor='none', facecolor='white', alpha=0.75,transform=imgAX.transAxes )
            if len(odomIntPos) > 1:
                odomIntEnd = odomIntPos[-1]
                offRot = currOdomIntRot.T
#                odomIntPath = np.array([np.dot(offRot,x-odomIntEnd) for x in reversed(odomIntPos)])
                odomIntPath = np.array([x-odomIntEnd for x in reversed(odomIntPos)])
                relPts = odomBoxOrigin.reshape(1,2) + np.multiply(odomBoxScale.reshape(1,2),odomIntPath)
                lines = imgAX.plot( relPts[:,0], relPts[:,1], linewidth='5.0', color='blue', transform=imgAX.transAxes )
                for line in lines:
                    line.set_clip_box(mptrans.TransformedBbox(mptrans.Bbox(np.array([odomBoxMin,odomBoxMax])),imgAX.transAxes)) 
#            imgAX.arrow(odomBoxOrigin[0], odomBoxOrigin[1], (0.005/odomBoxScale[0])*np.sin(curr_obs['odom_int_ang']), (0.005/odomBoxScale[1])*np.cos(curr_obs['odom_int_ang']),
#                        linewidth='5.0', color='blue', head_starts_at_zero=True, length_includes_head=True, head_width=0.02, 
#                        transform=imgAX.transAxes)
            imgAX.text(0.15, 0.9, 'Visual Odometry', ha='center', va='baseline',
                       bbox={'facecolor':'white','alpha':0.5,'edgecolor':'none'},
                       fontsize=14, fontweight='bold', transform=imgAX.transAxes)

            # Display the map posterior
            rightAX.cla()
            rightAX.set_xlim((dispBoxMin[0],dispBoxMax[0]))
            rightAX.set_ylim((dispBoxMin[1],dispBoxMax[1]))
            rightAX.set_xticks(majorXTicks[np.logical_and(majorXTicks >= dispBoxMin[0],majorXTicks <= dispBoxMax[0])])
            rightAX.set_yticks(majorYTicks[np.logical_and(majorYTicks >= dispBoxMin[1],majorYTicks <= dispBoxMax[1])])
            rightAX.set_xticks(minorXTicks[np.logical_and(minorXTicks >= dispBoxMin[0],minorXTicks <= dispBoxMax[0])],minor=True)
            rightAX.set_yticks(minorYTicks[np.logical_and(minorYTicks >= dispBoxMin[1],minorYTicks <= dispBoxMax[1])],minor=True)
            rightAX.set_xticklabels([]) 
            rightAX.set_yticklabels([])
            rightAX.grid(True,'major')
            rightAX.text(0.5, 0.95, 'Inference Result', ha='center', va='baseline', 
                         bbox={'facecolor':'white','alpha':0.5,'edgecolor':'none'},
                         fontsize=14, fontweight='bold', transform=rightAX.transAxes)
            
            post.displayPosterior(mapgraph, mapdynamics, fig, rightAX, gtX = currGtX, gtPath = prevGTs, odomPosPath = prevOdomPts, modes = topModes)

            rightAX.text(0.9,0.05,'{0:.0f}s'.format(t),transform=rightAX.transAxes) 

            # Update the box on the full map
            boxLines = draw_box(leftAX,dispBoxPos,dispBoxSz)

            fname = get_outfile_path(options,'postDisplay-frame{0:04}.{1}'.format(tInd+1,options.img_format))
            fig.subplots_adjust(left=0,right=1,top=1,bottom=0,wspace=0,hspace=0)
            fig.savefig(fname,bbox_inches='tight',dpi = 0.25*displayDPI)
            for l in boxLines: l.remove()
        
        modes_pkl_file.close()
        stats_pkl_file.close()
        if baseData['mpi']:
            for pfile in post_pkl_files:
                pfile.close()
        else:
            post_pkl_file.close()
            
        fig.clf()

def do_display(options,args,cropRegion = False,trackGT = True):
    regionSz = 750.0/1000.0
    boxPosLag = 0.85
    boxSzLag = 0.9

    load_config_file(options,args)

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    mapgraph,mapVersion = load_mapgraph(options)
    fullMapSz = mapgraph.posMax - mapgraph.posMin
    fullMapCenter = 0.5*(mapgraph.posMax + mapgraph.posMin)

    majorTickSz = 500.0/1000.0
    while np.any((fullMapSz/majorTickSz) > 12):
        majorTickSz *= 2.0
    minorTickSz = majorTickSz/5.0

    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']
    latlon = np.empty(2)

    sortedStreets = sorted(mapgraph.nodes_iter())

    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    mapdynamics = baseData['mapdynamics']
    start_frame = baseData['start_frame']
    stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')
    modes_pkl_file = open(get_outfile_path(options,'modes.p'),'rb')
    nmsParams = pickle.load(modes_pkl_file)

    # Open the file with the posteriors
    if baseData['mpi']:
        post_pkl_files = [ open(get_outfile_path(options,'posteriors-rank{0:03}.p'.format(crank)),'rb') for crank in range(baseData['size']) ]
        post = None
        for pfile in post_pkl_files:
            if post == None:
                post = pickle.load(pfile)
            else:
                post.update(pickle.load(pfile))
    else:
        post_pkl_file = open(get_outfile_path(options,'posteriors.p'),'rb')
        post = pickle.load(post_pkl_file)

#    if cropRegion and options.gps_data == None:
#        print 'Need GPS data to crop map region, not producing cropped displays.'
#        return

    drawBoxes = []
    # Open the GPS data
    if options.gps_data != None:
        if cropRegion:
            pathMin = np.inf * np.ones(2)
            pathMax = -np.inf * np.ones(2)

            stateIter = iter(open(options.gps_data,'rt'))
            stateInd = -1
            for statet in stateIter:
                stateInd += 1
                lineParts = statet.split()
                latlon[0] = float(lineParts[0])
                latlon[1] = float(lineParts[1])
                gtPos = mercatorProj(latlon,projScale) - ptCoordOrigin

                pathMin = np.minimum(pathMin,gtPos.reshape(2))
                pathMax = np.maximum(pathMax,gtPos.reshape(2))
                if trackGT and options.sequence_id in sequenceFrames and  stateInd in sequenceFrames[options.sequence_id]:
                    drawBoxes.append((gtPos,np.array([regionSz,regionSz]))) 

            if trackGT:
                gtmapCenter = 0.5*(pathMin + pathMax)
                regSize = 2*np.max(np.maximum(fullMapCenter - pathMin, pathMax - fullMapCenter))
                gtmapSz = np.copy(fullMapSz)
                gtmapSz[:] = regSize + 15.0/1000.0
            else:
                mapCenter = 0.5*(pathMin + pathMax)
                regSize = 2*np.max(np.maximum(mapCenter - pathMin, pathMax - mapCenter))
                mapSz = (regSize + 15.0/1000.0)*np.ones_like(mapCenter)

        stateIter = iter(open(options.gps_data,'rt'))
    else:
        if cropRegion:
            if trackGT:
                for curr_obs in baseData['obs']:
                    t = curr_obs['t']
                    tInd = curr_obs['tInd']
                    curr_data_dt = curr_obs['dt']
                    if tInd < start_frame:
                        continue

                    topModes = pickle.load(modes_pkl_file)
            
                    if tInd+1 in sequenceFrames[options.sequence_id]:
                        modesMin = mapgraph.posMax
                        modesMax = mapgraph.posMin
                        for (p,curr_street,curr_pos,curr_heading) in topModes:
                            modesMin = np.minimum(modesMin.reshape(2),curr_pos.reshape(2))
                            modesMax = np.maximum(modesMax.reshape(2),curr_pos.reshape(2))
            
                        # Update the display box based on the current set of modes
                        dispBoxPos = boxPosLag*dispBoxPos + (1.0-boxPosLag)*0.5*(modesMin + modesMax)
                        dispBoxSz = boxSzLag*dispBoxSz + (1.0-boxSzLag)*(modesMax - modesMin)
                        dispBoxSz[0:2] = np.maximum(np.max(dispBoxSz),regionSz)
                        
                        # Adjust it so that it doesn't move outside the bounds of the map region
                        adjust_box(dispBoxPos,dispBoxSz,fullMapCenter,fullMapSz)
            
                        drawBoxes.append((dispBoxPos,drawBoxSz)) 

                modes_pkl_file = open(get_outfile_path(options,'modes.p'),'rb')

                gtmapCenter = np.copy(fullMapCenter)
                gtmapSz = np.copy(fullMapSz)
                
                dispBoxPos = np.copy(fullMapCenter)
                dispBoxSz = np.copy(fullMapSz)
            else:
                mapCenter = np.copy(fullMapCenter)
                mapSz = np.copy(fullMapSz)
            
    if cropRegion:
        fig = plt.figure(dpi = displayDPI,figsize=displayCroppedFigSz)
    else:
        fig = plt.figure(dpi = displayDPI,figsize=displayFigSz)

    majorXTicks = np.arange(mapgraph.posMin[0] - 1.0,mapgraph.posMax[0] + 1.0, majorTickSz)
    majorYTicks = np.arange(mapgraph.posMin[1] - 1.0,mapgraph.posMax[1] + 1.0, majorTickSz)
    minorXTicks = np.arange(mapgraph.posMin[0] - 1.0,mapgraph.posMax[0] + 1.0, minorTickSz)
    minorYTicks = np.arange(mapgraph.posMin[1] - 1.0,mapgraph.posMax[1] + 1.0, minorTickSz)

    frameCount = 0
    if options.resume <= 0 and (frameCount%size == rank):        
        fig.clf()
        fig.subplots_adjust(left=0,right=1,top=1,bottom=0,wspace=0,hspace=0)
        ax = fig.add_subplot(111)
        if cropRegion and trackGT:
            cmin = fullMapCenter - 0.5*np.max(fullMapSz)
            cmax = fullMapCenter + 0.5*np.max(fullMapSz)
        else:
            cmin = mapgraph.posMin
            cmax = mapgraph.posMax
        ax.set_xlim((cmin[0],cmax[0]))
        ax.set_ylim((cmin[1],cmax[1]))
        ax.set_xticks(majorXTicks[np.logical_and(majorXTicks >= cmin[0],majorXTicks <= cmax[0])])
        ax.set_yticks(majorYTicks[np.logical_and(majorYTicks >= cmin[1],majorYTicks <= cmax[1])])
        ax.set_xticks(minorXTicks[np.logical_and(minorXTicks >= cmin[0],minorXTicks <= cmax[0])],minor=True)
        ax.set_yticks(minorYTicks[np.logical_and(minorYTicks >= cmin[1],minorYTicks <= cmax[1])],minor=True)
        ax.set_xticklabels([]) 
        ax.set_yticklabels([])
        ax.grid(True,'major')
        
        post.displayPosterior(mapgraph, mapdynamics, fig, ax)
       
        if cropRegion:
            ax.text(0.1,0.9,'Sequence {0}'.format(options.sequence_id),transform=ax.transAxes)
 
        if cropRegion and not trackGT:
            draw_box(ax,mapCenter,mapSz)
            fname = get_outfile_path(options,'postDisplay-crop{0:04}.{1}'.format(0,options.img_format))
        elif cropRegion and trackGT:
            for (boxPos,boxSz) in drawBoxes:
                adjust_box(boxPos,boxSz,gtmapCenter,gtmapSz)
                draw_box(ax,boxPos,boxSz)
            fname = get_outfile_path(options,'postDisplay-croptrack{0:04}.{1}'.format(0,options.img_format))
        else:
            fname = get_outfile_path(options,'postDisplay{0:04}.{1}'.format(0,options.img_format))
        fig.savefig(fname,bbox_inches='tight',dpi = displayDPI)

    odomIntPos = []
    if options.gps_data != None:
        prevGTs = []
        prevOdomPts = []
    else:
        prevGTs = None
        prevOdomPts = None
    
    stateInd = -1
    compTimes = []
    wallTimes = []
    dataTimes = []
    logEvidences = []
    numModes = []
    dataDTs = []
    for curr_obs in baseData['obs']:
        t = curr_obs['t']
        tInd = curr_obs['tInd']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']
        
        if tInd < start_frame:
            continue

        # Read GPS data to get the ground truth position
        if options.gps_data != None:
            while stateInd < tInd:
                statet = stateIter.next()
                stateInd += 1
                lineParts = statet.split()
                latlon[0] = float(lineParts[0])
                latlon[1] = float(lineParts[1])
                heading = np.pi/2.0 - float(lineParts[5])
                gtPos = mercatorProj(latlon,projScale) - ptCoordOrigin

                if stateInd == 0:
                    # Use the first point to orient the odometry (currently not used)
                    odomIntRot = np.array([(np.cos(heading), np.sin(heading)), (-np.sin(heading),np.cos(heading))])
                    odomIntOrigin = np.copy(gtPos)
                currGtX = {'latlon':copy(latlon)}

                if stateInd >= start_frame:
                    prevGTs.append(deepcopy(currGtX))

            prevOdomPts.append(odomIntOrigin + np.dot(odomIntRot,curr_obs['odom_int_pos']))
        else:
            currGtX = None

        odomIntPos.append(curr_obs['odom_int_pos'])
        currOdomIntRot = np.array([( np.cos(curr_obs['odom_int_ang']), np.sin(curr_obs['odom_int_ang'])), \
                                   (-np.sin(curr_obs['odom_int_ang']), np.cos(curr_obs['odom_int_ang']))])

        topModes = pickle.load(modes_pkl_file)
        stepData = pickle.load(stats_pkl_file)
        numModes.append(len(topModes))
        logEvidences.append(stepData['logEvidence'])
        dataTimes.append(t)
        dataDTs.append(curr_data_dt)
        compTimes.append(stepData['iter_dt'])
        wallTimes.append(stepData['wall_dt'])

        if cropRegion and trackGT and options.gps_data == None:
            modesMin = mapgraph.posMax
            modesMax = mapgraph.posMin
            for (p,curr_street,curr_pos,curr_heading) in topModes:
                modesMin = np.minimum(modesMin.reshape(2),curr_pos.reshape(2))
                modesMax = np.maximum(modesMax.reshape(2),curr_pos.reshape(2))

            # Update the display box based on the current set of modes
            dispBoxPos = boxPosLag*dispBoxPos + (1.0-boxPosLag)*0.5*(modesMin + modesMax)
            dispBoxSz = boxSzLag*dispBoxSz + (1.0-boxSzLag)*(modesMax - modesMin)
            dispBoxSz[0:2] = np.maximum(np.max(dispBoxSz),regionSz)
            
            # Adjust it so that it doesn't move outside the bounds of the map region
            adjust_box(dispBoxPos,dispBoxSz,fullMapCenter,fullMapSz)

            dispBoxMin = dispBoxPos - 0.5*dispBoxSz
            dispBoxMax = dispBoxPos + 0.5*dispBoxSz

        # frameCount is used to keep track of which worker is processing which frame
        frameCount += 1
        if tInd < options.resume or not (frameCount%size == rank):
            continue

        if size == 1:
            if baseData['states'] == None:
                print '\n\ntInd = {0}, t = {2}, obst = {1}'.format(tInd,yt.T,t)
            else:
                print '\n\ntInd = {0}, t = {4}, obst = {1}, street = {2}, state = {3}'.format(tInd,yt.T,statet['street'],statet['state'].T,t)

        if baseData['mpi']:
            for (frameRef,pfile) in izip(stepData['post_pos_ref'],post_pkl_files):
                pfile.seek(frameRef)
                post.update(pickle.load(pfile))
        else:
            post_pkl_file.seek(stepData['post_pos_ref'][0])
            post = pickle.load(post_pkl_file)

        if not options.disable_postprint:
            for vt in sortedStreets:
                if vt != SINK_ROAD and np.isfinite(post.logV[vt]):
                    cmu = post.Theta[vt].computeMean()
                    print 'p[{0}] = {1}, mu = {2}'.format(vt, float(np.exp(post.logV[vt])), cmu.reshape(4))

        if not options.mpi and baseData['mpi']:
            avg_dts = stepData['avg_dts']
            print 'Load %: {0}, Max imbalance {1}%'.format(100.0*avg_dts/np.sum(avg_dts),100.0*(np.max(avg_dts) - np.min(avg_dts))/np.sum(avg_dts))
    
        # Display the map posterior
        if trackGT and cropRegion:
            if options.gps_data != None:
                cmapCenter = np.copy(gtPos)
                cmapSz = np.empty_like(fullMapCenter)
                cmapSz[0] = regionSz
                cmapSz[1] = regionSz
                adjust_box(cmapCenter,cmapSz,gtmapCenter,gtmapSz)
            else:
                cmapCenter = dispBoxPos
                cmapSz = dispBoxSz
        elif cropRegion:
            cmapCenter = mapCenter
            cmapSz = mapSz
        else:
            cmapCenter = fullMapCenter
            cmapSz = fullMapSz

        fig.clf()
        fig.subplots_adjust(left=0,right=1,top=1,bottom=0,wspace=0,hspace=0)
        ax = fig.add_subplot(111)
        ax.set_xlim((cmapCenter[0] - 0.5*cmapSz[0],cmapCenter[0] + 0.5*cmapSz[0]))
        ax.set_ylim((cmapCenter[1] - 0.5*cmapSz[1],cmapCenter[1] + 0.5*cmapSz[1]))
        ax.set_xticks(majorXTicks[np.logical_and(majorXTicks - cmapCenter[0] + 0.5*cmapSz[0] >= 0,majorXTicks - cmapCenter[0] - 0.5*cmapSz[0] <= 0)])
        ax.set_yticks(majorYTicks[np.logical_and(majorYTicks - cmapCenter[1] + 0.5*cmapSz[1] >= 0,majorYTicks - cmapCenter[1] - 0.5*cmapSz[1] <= 0)])
        ax.set_xticks(minorXTicks[np.logical_and(minorXTicks - cmapCenter[0] + 0.5*cmapSz[0] >= 0,minorXTicks - cmapCenter[0] - 0.5*cmapSz[0] <= 0)],minor=True)
        ax.set_yticks(minorYTicks[np.logical_and(minorYTicks - cmapCenter[1] + 0.5*cmapSz[1] >= 0,minorYTicks - cmapCenter[1] - 0.5*cmapSz[1] <= 0)],minor=True)
        ax.set_xticklabels([]) 
        ax.set_yticklabels([])
        ax.grid(True,'major')
        
        post.displayPosterior(mapgraph, mapdynamics, fig, ax, gtX = currGtX, gtPath = prevGTs, odomPosPath = prevOdomPts, odomIntPos = [np.dot(currOdomIntRot.T,x) for x in odomIntPos],modes = topModes)

        if cropRegion:
            ax.text(0.1,0.9,'{0:.0f}s'.format(t),transform=ax.transAxes) 

        if cropRegion and not trackGT:
            fname = get_outfile_path(options,'postDisplay-crop{0:04}.{1}'.format(tInd+1,options.img_format))
        elif cropRegion and trackGT:
            fname = get_outfile_path(options,'postDisplay-croptrack{0:04}.{1}'.format(tInd+1,options.img_format))
        else:
            fname = get_outfile_path(options,'postDisplay{0:04}.{1}'.format(tInd+1,options.img_format))
        fig.savefig(fname,bbox_inches='tight',dpi = displayDPI)
    
    modes_pkl_file.close()
    stats_pkl_file.close()
    if baseData['mpi']:
        for pfile in post_pkl_files:
            pfile.close()
    else:
        post_pkl_file.close()

def load_stats(options,args):
    if args != None:
        load_config_file(options,args)

    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    start_frame = baseData['start_frame']
    stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')
    modes_pkl_file = open(get_outfile_path(options,'modes.p'),'rb')
    nmsParams = pickle.load(modes_pkl_file)

    compTimes = []
    wallTimes = []
    dataTimes = []
    logEvidences = []
    numModes = []
    dataDTs = []
    for curr_obs in baseData['obs']:
        t = curr_obs['t']
        tInd = curr_obs['tInd']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']
        
        if tInd < start_frame:
            continue

        topModes = pickle.load(modes_pkl_file)
        stepData = pickle.load(stats_pkl_file)
        numModes.append(len(topModes))
        logEvidences.append(stepData['logEvidence'])
        dataTimes.append(t)
        dataDTs.append(curr_data_dt)
        compTimes.append(stepData['iter_dt'])
        wallTimes.append(stepData['wall_dt'])

    modes_pkl_file.close()
    stats_pkl_file.close()

    return {'numModes':numModes,'logEvidences':logEvidences,'dataTimes':dataTimes, \
            'dataDTs':dataDTs,'compTimes':compTimes,'wallTimes':wallTimes,'baseData':baseData}


def do_plot_stats(options,args):
    stats = load_stats(options,args)
   
    dataTimes = stats['dataTimes']
    wallTimes = stats['wallTimes']
    numModes = stats['numModes']
    logEvidences = stats['logEvidences']
    compTimes = stats['compTimes'] 
    dataDTs = stats['dataDTs'] 
    baseData = stats['baseData'] 

    fig = plt.figure(dpi = plotDPI,figsize=(plotFigSize[0],2*plotFigSize[1]))

    # Display computation time
    fig.clf()
    ax = fig.add_subplot(2,1,1)
    ax.plot(dataTimes,np.cumsum(wallTimes),'b-', label = '{0} cores'.format(baseData['size']))
    ax.plot([dataTimes[0],dataTimes[-1]],[dataTimes[0],dataTimes[-1]],'--', label = 'Realtime')
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Computation time (seconds)')
    ax.set_title('Cumulative computation time')
    ax.legend()
    ax = fig.add_subplot(2,1,2)
    ax.plot(dataTimes,wallTimes,'b-', label = '{0} cores'.format(baseData['size']))
    ax.plot(dataTimes,dataDTs,'--', label = 'Frame rate')
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Computation time (seconds)')
    ax.set_title('Per frame computation time')
    ax.legend()
    fname = get_outfile(options,'timing.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)

    # Plot log evidence over time
    fig.clf()
    ax = fig.add_subplot(2,1,1)
    ax.plot(dataTimes,numModes)
#    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('# of modes')
    ax.set_title('Number of modes')
        
    ax = fig.add_subplot(2,1,2)
    ax.plot(dataTimes,logEvidences)
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('log(P(y_t|y_{1:t-1}))')
    ax.set_title('log evidence')
    fname = get_outfile(options,'evidence.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)
        

def do_make_movie(options,args):
    use_mencoder = True
    load_config_file(options,args)
    if use_mencoder:
        command = ('mencoder',
    #               'mf://' + get_outfile_path(options,'postDisplay[0-9]*.{0}'.format(options.img_format)),
                   'mf://' + get_outfile_path(options,'postDisplay-frame[0-9]*.{0}'.format(options.img_format)),
                   '-mf', 'type=png:fps=5',
    #               '-of', 'avi',
    #               '-ovc', 'xvid',
    #               '-xvidencopts', 'fixed_quant=4',
    #               '-xvidencopts', 'fixed_quant=20:autoaspect:max_key_interval=2:vhq=2:bvhq=1:trellis:hq_ac:chroma_me:chroma_opt:quant_type=mpeg:threads=8', 
                   '-of', 'lavf',
                   '-lavfopts', 'format=mp4',
                   '-ovc', 'x264',
    #               '-x264encopts', 'bitrate=400:vbv_maxrate=700:vbv_bufsize=1500:me=umh:crf=18:nocabac:global_header:trellis=1:threads=8',
                   '-x264encopts', 'crf=4:nofast_pskip:nodct_decimate:nocabac:global_header:threads=8',
                   '-oac', 'copy',
#                   '-o', get_outfile(options,'postMovie.avi'))
                   '-o', get_outfile(options,'postMovie.mp4'))
    else:
        #ffmpeg -r 10 -i %06d.png -vcodec mpeg4 -vtag xvid -b 2000k video.avi
        command = ('ffmpeg', '-r', '5', '-y',
                   '-i', get_outfile_path(options,'postDisplay-frame%03d0.{0}'.format(options.img_format)),
                   '-vcodec', 'mpeg4', '-vtag', 'xvid',
                   '-b', '1500k', '-s', '800x600', 
                   get_outfile(options,'postMovie.avi'))
        
    subprocess.call(command)

def do_plot_crop_errors(options,args):
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    cropSizes = [float(a) for a in args[0].split(' ')]
    cfgFiles = args[1:]

    modeInfo = {}
    statsInfo = {}
    sources = set()
    sequences = set()
    for i in range(rank,len(cfgFiles)+size,size):
        if i >= len(cfgFiles):
            break
        
        cOptions = copy(options)
        load_config_file(cOptions,cfgFiles[i])
        sources.add(cOptions.data_source)
        sequences.add(cOptions.sequence_id)

        for (cInd,cropSize) in enumerate(cropSizes):
            options.crop_size = cropSize
            modeInfo[(cOptions.data_source,cropSize,cOptions.sequence_id)] = do_load_errors(copy(options),cfgFiles[i])
            statsInfo[(cOptions.data_source,cropSize,cOptions.sequence_id)] = load_stats(copy(options),cfgFiles[i])
            print 'Loaded {0}, crop {1}'.format(cfgFiles[i],cropSize)

    if options.mpi:
        allModeInfo = comm.gather((modeInfo,sources,sequences,statsInfo))
        if rank == 0:
            for cmode in allModeInfo:
                modeInfo.update(cmode[0])
                sources |= cmode[1]
                sequences |= cmode[2]
                statsInfo.update(cmode[3])

    if rank != 0:
        return

    fig = plt.figure(figsize=plotFigSize,dpi=plotDPI)
    fig.subplots_adjust(bottom = 0.15)

    sources = sorted(sources)
    sequences = sorted(sequences)
    
    groupErrors = []
    for srcName in sources:
        seqErrs = np.empty((len(cropSizes),len(topErrNs),len(sequences)+1,2,2))
        localizeTimes = np.empty((len(cropSizes),len(sequences)+1))
        localizeDists = np.empty((len(cropSizes),len(sequences)+1,2))
        for (cInd,cSize) in enumerate(cropSizes):
            localizedFlags = []
            totalLocalizedErrs = 0
            totalLocalizedFrames = 0
            totalErrs = 0
            totalFrames = 0
            for (seqInd,seqId) in enumerate(sequences):
                if modeInfo[(srcName,cSize,seqId)][1] == None:
                    print 'No GPS data for {0}.'.format(seqId)
                    continue
                currErrs = np.array(modeInfo[(srcName,cSize,seqId)][1]['absGPSErr'])
                moveDists = np.array(modeInfo[(srcName,cSize,seqId)][6])
                localizedFrameFlags = np.array(modeInfo[(srcName,cSize,seqId)][2])
                totalErrs += np.sum(currErrs,1)
                totalFrames += currErrs.shape[1]
                totalLocalizedErrs += np.sum(currErrs[:,localizedFrameFlags,:],1)
                totalLocalizedFrames += np.sum(localizedFrameFlags)

                localizedInd = modeInfo[(srcName,cSize,seqId)][3]
                localizeTimes[cInd,seqInd] = modeInfo[(srcName,cSize,seqId)][4]
                localizeDists[cInd,seqInd,:] = np.sum(np.abs(moveDists[:localizedInd,:]),0)
                localizedFlags.append(np.isfinite(modeInfo[(srcName,cSize,seqId)][4]))
    
            localizedFlags.append(False)
            localizedFlags = np.array(localizedFlags)
            localizeTimes[cInd,len(sequences)] = np.mean(localizeTimes[cInd,localizedFlags])
            localizeDists[cInd,len(sequences),:] = np.mean(localizeDists[cInd,localizedFlags,:],0).reshape((1,1,2))
        
        groupErrors.append((np.copy(localizeTimes[:,-1]),np.copy(localizeDists[:,-1,:])))

        fig.clf()
        ax = fig.add_subplot(111)
        ax.plot(cropSizes,localizeTimes[:,-1],label = 'avg', linewidth=4.0, linestyle='--')
        for (seqInd,seqId) in enumerate(sequences):
            ax.plot(cropSizes,localizeTimes[:,seqInd],label = seqId,linewidth=2.0,alpha=0.5)
        ax.set_xlabel('Initial Region Size (km)')
        ax.set_ylabel('Time to Localize (s)')
        ax.legend()
        fname = os.path.join(options.out_dir,'{0}-CroppedLocalizeTime.{1}'.format(srcName,options.img_format))
        #plt.tight_layout()
        fig.savefig(fname,dpi=plotDPI)

        fig.clf()
        ax = fig.add_subplot(111)
        ax.plot(cropSizes,1000.0*localizeDists[:,-1,1],label = 'avg', linewidth=4.0, linestyle='--')
        for (seqInd,seqId) in enumerate(sequences):
            ax.plot(cropSizes,1000.0*localizeDists[:,seqInd,1],label = seqId,linewidth=2.0,alpha=0.5)
        ax.set_xlabel('Initial Region Size (km)')
        ax.set_ylabel('Distance to Localize (m)')
        ax.legend()
        fname = os.path.join(options.out_dir,'{0}-CroppedLocalizeDist.{1}'.format(srcName,options.img_format))
        #plt.tight_layout()
        fig.savefig(fname,dpi=plotDPI)

        fig.clf()
        ax = fig.add_subplot(111)
        ax.plot(cropSizes,(180.0/np.pi)*localizeDists[:,-1,0],label = 'avg', linewidth=4.0, linestyle='--')
        for (seqInd,seqId) in enumerate(sequences):
            ax.plot(cropSizes,(180.0/np.pi)*localizeDists[:,seqInd,0],label = seqId,linewidth=2.0,alpha=0.5)
        ax.set_xlabel('Initial Region Size (km)')
        ax.set_ylabel('Angular Change to Localize (deg)')
        ax.legend()
        fname = os.path.join(options.out_dir,'{0}-CroppedLocalizeAng.{1}'.format(srcName,options.img_format))
        #plt.tight_layout()
        fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(cropSizes,errs[0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Initial Region Size (km)')
    ax.set_ylabel('Time to Localize (s)')
    ax.legend(loc='upper left')
    fname = os.path.join(options.out_dir,'CroppedLocalizeTime.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(cropSizes,(180.0/np.pi)*errs[1][:,0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Initial Region Size (km)')
    ax.set_ylabel('Distance to Localize (m)')
    ax.legend(loc='upper left')
    fname = os.path.join(options.out_dir,'CroppedLocalizeDist.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(cropSizes,1000.0*errs[1][:,1],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Initial Region Size (km)')
    ax.set_ylabel('Distance to Localize (m)')
    ax.legend(loc='upper left')
    fname = os.path.join(options.out_dir,'CroppedLocalizeAng.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.savefig(fname,dpi=plotDPI)

def do_plot_natthresh_errors(options,args):
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    natThreshes = [float(a) for a in args[0].split(' ')]
    cfgFiles = args[1:]

    modeInfo = {}
    statsInfo = {}
    sources = set()
    sequences = set()
    for i in range(rank,len(cfgFiles)+size,size):
        if i >= len(cfgFiles):
            break
        
        cOptions = copy(options)
        load_config_file(cOptions,cfgFiles[i])
        sources.add(cOptions.data_source)
        sequences.add(cOptions.sequence_id)

        for (cInd,natThresh) in enumerate(natThreshes):
            options.simplify_threshold = natThresh
            modeErrs = do_load_errors(copy(options),cfgFiles[i])
            cStats = load_stats(copy(options),cfgFiles[i])
            modeInfo[(cOptions.data_source,natThresh,cOptions.sequence_id)] = modeErrs
            statsInfo[(cOptions.data_source,natThresh,cOptions.sequence_id)] = cStats

    if options.mpi:
        allModeInfo = comm.gather((modeInfo,sources,sequences,statsInfo))
        if rank == 0:
            for cmode in allModeInfo:
                modeInfo.update(cmode[0])
                sources |= cmode[1]
                sequences |= cmode[2]
                statsInfo.update(cmode[3])

    if rank != 0:
        sys.exit()
    
    sources = sorted(sources)
    sequences = sorted(sequences)
    
    groupErrors = []
    for srcName in sources:
        seqErrs = np.empty((len(natThreshes),1,len(sequences)+1,2,2))
        compTimes = np.empty((len(natThreshes),len(sequences)+1))
        localizeTimes = np.empty((len(natThreshes),len(sequences)+1))
        localizeDists = np.empty((len(natThreshes),len(sequences)+1,2))
        for (cInd,natThresh) in enumerate(natThreshes):
            localizedFlags = []
            totalLocalizedErrs = 0
            totalLocalizedFrames = 0
            totalErrs = 0
            totalFrames = 0
            compFrames = np.empty((len(sequences)))
            for (cfgI,cfgFile) in enumerate(sequences):
                if modeInfo[(srcName,natThresh,cfgFile)][1] == None:
                    print 'No GPS data for {0}.'.format(cfgFile)
                    continue
                compTimes[cInd,cfgI] = np.mean(statsInfo[(srcName,natThresh,cfgFile)]['wallTimes'])
                currErrs = np.array(modeInfo[(srcName,natThresh,cfgFile)][1]['absGPSErr'])
                moveDists = np.array(modeInfo[(srcName,natThresh,cfgFile)][6])
                localizedFrameFlags = np.array(modeInfo[(srcName,natThresh,cfgFile)][2])
                #for (i,cN) in enumerate(topErrNs):
                for (i,cN) in enumerate(topErrNs[0:1]):
                    posErr,angErr = 1000.0*np.mean(currErrs[i,:,0]), (180.0/np.pi)*np.mean(currErrs[i,:,1])
                    posLocalErr,angLocalErr = 1000.0*np.mean(currErrs[i,localizedFrameFlags,0]), (180.0/np.pi)*np.mean(currErrs[i,localizedFrameFlags,1])
                    seqErrs[cInd,i,cfgI,0,0] = posErr
                    seqErrs[cInd,i,cfgI,1,0] = angErr
                    seqErrs[cInd,i,cfgI,0,1] = posLocalErr
                    seqErrs[cInd,i,cfgI,1,1] = angLocalErr
                totalErrs += np.sum(currErrs,1)
                totalFrames += currErrs.shape[1]
                compFrames[cfgI] = currErrs.shape[1]
                totalLocalizedErrs += np.sum(currErrs[:,localizedFrameFlags,:],1)
                totalLocalizedFrames += np.sum(localizedFrameFlags)

                localizedInd = modeInfo[(srcName,natThresh,cfgFile)][3]
                localizeTimes[cInd,cfgI] = modeInfo[(srcName,natThresh,cfgFile)][4]
                localizeDists[cInd,cfgI,:] = np.sum(np.abs(moveDists[:localizedInd,:]),0)
                localizedFlags.append(np.isfinite(modeInfo[(srcName,natThresh,cfgFile)][4]))
    
            localizedFlags.append(False)
            localizedFlags = np.array(localizedFlags)
            localizeTimes[cInd,len(sequences)] = np.mean(localizeTimes[cInd,localizedFlags])
            localizeDists[cInd,len(sequences),:] = np.mean(localizeDists[cInd,localizedFlags,:],0).reshape((1,1,2))
            compTimes[cInd,len(sequences)] = np.sum(np.multiply(compFrames,compTimes[cInd,0:len(sequences)]))/totalFrames
            
#            for (i,cN) in enumerate(topErrNs):
            for i in range(1):
                posErr,angErr = 1000.0*(totalErrs[i,0]/totalFrames), (180.0/np.pi)*(totalErrs[i,1]/totalFrames)
                posLocalErr,angLocalErr = 1000.0*(totalLocalizedErrs[i,0]/totalLocalizedFrames), (180.0/np.pi)*(totalLocalizedErrs[i,1]/totalLocalizedFrames)
                seqErrs[cInd,i,len(sequences),0,0] = posErr
                seqErrs[cInd,i,len(sequences),1,0] = angErr
                seqErrs[cInd,i,len(sequences),0,1] = posLocalErr
                seqErrs[cInd,i,len(sequences),1,1] = angLocalErr
        
        groupErrors.append((localizeTimes[:,-1],localizeDists[:,-1,:], seqErrs[:,0,-1,:,1], compTimes[:,-1]))

        fig = plt.figure(figsize=plotFigSize,dpi=plotDPI)
#        fig.subplots_adjust(bottom = 0.15)
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,localizeTimes[:,-1],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,localizeTimes[:,cfgI],label = (os.path.basename(cfgFile).split('-'))[0],linewidth=2.0,alpha=0.5)
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_ylabel('Time to Localize (s)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyLocalizeTime.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)
#
#        fig.clf()
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,compTimes[:,-1],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,compTimes[:,cfgI],label = cfgFile,linewidth=2.0,alpha=0.5)
#        ax.set_ylabel('Computation Time Per Frame (s)')
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyComputationTime.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)
#
#        fig.clf()
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,seqErrs[:,0,len(sequences),0,1],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,seqErrs[:,0,cfgI,0,1],label = cfgFile,linewidth=2.0,alpha=0.5)
#        ax.set_ylabel('Position Error (m)')
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyPosErr.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)
#
#        fig.clf()
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,seqErrs[:,0,len(sequences),1,1],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,seqErrs[:,0,cfgI,1,1],label = cfgFile,linewidth=2.0,alpha=0.5)
#        ax.set_ylabel('Angular Error (deg)')
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyAngErr.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)
#
#        fig.clf()
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,1000.0*localizeDists[:,-1,1],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,1000.0*localizeDists[:,cfgI,1],label = (os.path.basename(cfgFile).split('-'))[0],linewidth=2.0,alpha=0.5)
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_ylabel('Distance to Localize (m)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyLocalizeDist.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)
#
#        fig.clf()
#        ax = fig.add_subplot(111)
#        ax.plot(natThreshes,(180.0/np.pi)*localizeDists[:,-1,0],label = 'avg', linewidth=4.0, linestyle='--')
#        for (cfgI,cfgFile) in enumerate(sequences):
#            ax.plot(natThreshes,(180.0/np.pi)*localizeDists[:,cfgI,0],label = (os.path.basename(cfgFile).split('-'))[0],linewidth=2.0,alpha=0.5)
#        ax.set_xlabel('Simplify Threshold (nats)')
#        ax.set_ylabel('Angular Change to Localize (deg)')
#        ax.set_xscale('log',basex = 2)
#        ax.legend()
#        fname = os.path.join(options.out_dir,'{0}-SimplifyLocalizeAng.{1}'.format(srcName,options.img_format))
#        #plt.tight_layout()
#        fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,errs[0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_ylabel('Time to Localize (s)')
    ax.set_xscale('log',basex = 10)
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    fname = os.path.join(options.out_dir,'SimplifyLocalizeTime.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,errs[3],label = srcName,linewidth=4.0, linestyle='-')
    ax.plot([natThreshes[0],natThreshes[-1]], [1,1], label = 'Real-time', linewidth=4.0, linestyle='--', color='red')
    ax.set_ylabel('Computation Time Per Frame (s)')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_xscale('log',basex = 10)
    ax.set_ylim((0,1.2))
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyComputationTime.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)
    
    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.scatter(errs[3],errs[2][:,0],label = srcName)
    ax.set_xlabel('Computation Time Per Frame (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_xscale('log',basex = 10)
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyComputationTimePosErrScatter.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,(180.0/np.pi)*errs[1][:,0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_ylabel('Angular Change to Localize (deg)')
    ax.set_xscale('log',basex = 10)
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyLocalizeDist.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,1000.0*errs[1][:,1],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_ylabel('Distance to Localize (m)')
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyLocalizeAng.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,errs[2][:,0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_ylabel('Position Error (m)')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_xscale('log',basex = 10)
#    ax.set_xscale('log',basex = 2)
    ax.set_ylim((0,29))
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyPosErr.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(111)
    for (srcName,errs) in izip(sources,groupErrors):
        ax.plot(natThreshes,errs[2][:,0],label = srcName, linewidth=4.0, linestyle='-')
    ax.set_ylabel('Angular Error (deg)')
    ax.set_xlabel('Simplify Threshold (nats)')
    ax.set_xscale('log',basex = 10)
#    ax.set_xscale('log',basex = 2)
    ax.legend()
    ax.grid()
    fname = os.path.join(options.out_dir,'SimplifyAngErr.{0}'.format(options.img_format))
    #plt.tight_layout()
    fig.subplots_adjust(bottom=0.15)
    fig.savefig(fname,dpi=plotDPI)
    
def do_print_map_stats(options,args):
    mapSzSum = 0
    N = 0
    driveLenSum = 0
    for cfgFile in args:
        if cfgFile.endswith('.dcfg'):
            cOptions = copy(options)
            load_config_file(cOptions,cfgFile)
    
            mapgraph,mapVersion = load_mapgraph(cOptions)
        elif cfgFile.endswith('.p'):
            mapgraph = pickle.load(open(cfgFile,'rb'))
            mapVersion = None

        mapSz = mapgraph.posMax - mapgraph.posMin
        
        mapSzSum += mapSz
        N += 1
        driveLenSum += mapgraph.totalLength
        
        print '{0}:'.format(cfgFile)
        print '  size: {0} x {1}'.format(mapSz[0],mapSz[1])
        print '  drivable length: {0}'.format(mapgraph.totalLength)
#        print '  chance error: {0}'.format(np.sqrt(np.sum(np.power(mapgraph.posAvg))))
        print '  avg size: {0} x {1}, avg drivable length: {2}'.format(mapSzSum[0]/N,mapSzSum[1]/N,driveLenSum/N)
        

def do_plot_snr_errors(options,args):
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    snrVals = set()
    sequences = set()
    cfgFiles = args
    modeInfo = {}

    for i in range(rank,len(cfgFiles)+size,size):
        if i >= len(cfgFiles):
            break
        
        cOptions = copy(options)
        load_config_file(cOptions,cfgFiles[i])
        cModeErrs = do_load_errors(cOptions,cfgFiles[i])

        if cModeErrs[1] == None:
            print 'No GPS data for {0}.'.format(cfgFiles[i])
            continue

        snrVals.add(cOptions.snr)
        sequences.add(cOptions.sequence_id)
        
        ckey = (cOptions.snr,cOptions.sequence_id)
        if ckey not in modeInfo:
            modeInfo[ckey] = [ [] for i in range(4) ]

        currErrs = np.array(cModeErrs[1]['absGPSErr'])
        localizedFrameFlags = np.array(cModeErrs[2])
        localizedInd = cModeErrs[3]
        moveDists = np.array(cModeErrs[6])
        
        if np.isfinite(cModeErrs[4]):
            modeInfo[ckey][0].append(np.array(np.mean(cModeErrs[4])))
            modeInfo[ckey][1].append(np.sum(np.abs(moveDists[:localizedInd,:]),0).reshape(1,2))
            modeInfo[ckey][2].append(np.mean(currErrs[0,localizedFrameFlags,:],0).reshape(1,2))
            modeInfo[ckey][3].append(np.array(np.sum(localizedFrameFlags)))

    if options.mpi:
        comm.Barrier()
        allModeInfo = comm.gather((modeInfo,snrVals,sequences))
        if rank == 0:
            modeInfo = {}
            for cmode in allModeInfo:
                for (ckey,cval) in cmode[0].iteritems():
                    if ckey not in modeInfo:
                        modeInfo[ckey] = cval
                    else:
                        for i in range(len(cval)):
                            modeInfo[ckey][i] += cval[i]
                snrVals |= cmode[1]
                sequences |= cmode[2]

    if rank != 0:
        sys.exit()
    else:
        snrVals = sorted(snrVals)
        sequences = sorted(sequences)

        localizeStatsMean = np.empty((len(snrVals),5))
        localizeStatsStd = np.empty((len(snrVals),5))
        for (noiseInd,noiseVal) in enumerate(snrVals):
            localizeStats = [ [] for i in range(4) ]
            for (seqInd,seqId) in enumerate(sequences):
                ckey = (noiseVal,seqId)
                
                assert ckey in modeInfo,'Missing SNR/Sequence combination: {0}'.format(ckey)
                
                for i in range(len(modeInfo[ckey])):
                    localizeStats[i] += modeInfo[ckey][i]
                    
            ws = np.array(localizeStats[3],dtype=float).reshape((-1,1))
            ws = ws / float(np.sum(ws))
          
            localizeStatsMean[noiseInd,0] = np.sum(np.multiply(ws,np.array(localizeStats[0]).reshape((-1,1))))
            localizeStatsStd[noiseInd,0] = np.sqrt(np.sum(np.multiply(ws,np.power(np.array(localizeStats[0]).reshape((-1,1)) - localizeStatsMean[noiseInd,0],2.0))))


            localizeStatsMean[noiseInd,1:3] = np.mean(np.concatenate(localizeStats[1]),0).reshape((1,2))
            localizeStatsStd[noiseInd,1:3] = np.std(np.concatenate(localizeStats[1]),0).reshape((1,2))
#            localizeStatsMean[noiseInd,1:3] = np.sum(np.multiply(ws,np.concatenate(localizeStats[1]))).reshape(1,2)
#            localizeStatsStd[noiseInd,1:3] = np.sqrt(np.sum(np.multiply(ws,np.power(np.concatenate(localizeStats[1]) - localizeStatsMean[noiseInd,1:3],2.0)))).reshape(1,2)

            localizeStatsMean[noiseInd,3:5] = np.sum(np.multiply(ws,np.concatenate(localizeStats[2])),0).reshape(1,2)
            localizeStatsStd[noiseInd,3:5] = np.sqrt(np.sum(np.multiply(ws,np.power(np.concatenate(localizeStats[2]) - localizeStatsMean[noiseInd,3:5],2.0)),0)).reshape(1,2)
#            localizeStatsMean[noiseInd,0] = np.mean(np.array(localizeStats[0]))
#            localizeStatsStd[noiseInd,0] = np.std(np.array(localizeStats[0]))
#
#            localizeStatsMean[noiseInd,1:3] = np.mean(np.concatenate(localizeStats[1]))
#            localizeStatsStd[noiseInd,1:3] = np.std(np.concatenate(localizeStats[1]))
#
#            localizeStatsMean[noiseInd,3:5] = np.mean(np.concatenate(localizeStats[2]))
#            localizeStatsStd[noiseInd,3:5] = np.std(np.concatenate(localizeStats[2]))

        plotSNRVals = 1.0/(np.array(snrVals))
        
        plotParams = [ (0, 'Time to Localization (s)', 'LocalizeTime', 1.0), \
                       (1, 'Angular Distance to Localize (deg)','LocalizeAng', (180.0/np.pi)), \
                       (2, 'Distance to Localize (m)', 'LocalizeDist', 1000.0), \
                       (3, 'Position Error (m)', 'PosErr', 1000.0), \
                       (4, 'Angular Error (deg)', 'AngErr', (180.0/np.pi)) ]
        
        fig = plt.figure(figsize=plotFigSize,dpi=plotDPI)
        fig.subplots_adjust(bottom = 0.2)
        for (i,yLabel,fileLabel,yScale) in plotParams:
            fig.clf()
            ax = fig.add_subplot(111)
            ax.plot(plotSNRVals,yScale*localizeStatsMean[:,i],label = 'Mean', linewidth=4.0, linestyle='-')
            ax.fill_between(plotSNRVals, \
                            yScale*(localizeStatsMean[:,i] - localizeStatsStd[:,i]),\
                            yScale*(localizeStatsMean[:,i] + localizeStatsStd[:,i]), \
                            alpha=0.5,facecolor='gray',edgecolor='none',label='1 Std Dev')
            ax.set_ylabel(yLabel)
            ax.set_xlabel('Signal to Noise Ratio')
            ax.set_xscale('log',basex=10)
            fname = os.path.join(options.out_dir,'SNR{1}.{0}'.format(options.img_format,fileLabel))
            fig.savefig(fname,dpi=plotDPI)

def do_compute_errors(options,args,topErrN):
    localizationSeqFramesThresh = 5
    localizationModesThresh = 1
    localizationPosErrThresh = 20.0/1000.0
    localizationAngErrThresh = np.pi/2.0
    
    load_config_file(options,args)

    if options.gps_data == None:
        print 'GPS data is required to compute errors, skipping {0}'.format(args)
        return
    assert options.gps_data != None, 'GPS data is required to compute errors'

    mapgraph,mapVersion = load_mapgraph(options)
    mapSz = mapgraph.posMax - mapgraph.posMin

    # Get data file and statistics
    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    mapdynamics = baseData['mapdynamics']
    start_frame = baseData['start_frame']
    stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')

    # Open modes file
    modes_pkl_file = open(get_outfile_path(options,'modes.p'),'rb')
    nmsParams = pickle.load(modes_pkl_file)
    assert nmsParams['nmsNumModes'] >= np.max(topErrN)
    
    gtStates = load_gt_data(options)
    
    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']
    latlon = np.empty(2)

    odomIntPos = []
    
    stateInd = -1
    dataTimes = []
    moveDists = []
    modes = []
    localizedFlags = []
    errors = {'absGPSErr':[ [] for i in topErrN ], \
              'gpsErr':[ ], \
              'relErr':[ [] for i in topErrN ], \
              'absMapErr':[ [] for i in topErrN ]}
    seqLocalFrames = 0
    localized = False

    prevState = None
    for (frameCount,(curr_obs,gtState)) in enumerate(izip(baseData['obs'],gtStates)):
        t = curr_obs['t']
        tInd = curr_obs['tInd']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']
        
        if tInd < start_frame:
            prevState = gtState
            continue
        
        if not options.mpi:
            if baseData['states'] == None:
                print '\n\ntInd = {0}, t = {2}, obst = {1}'.format(tInd,yt.T,t)
            else:
                print '\n\ntInd = {0}, t = {4}, obst = {1}, street = {2}, state = {3}'.format(tInd,yt.T,statet['street'],statet['state'].T,t)

        stepData = pickle.load(stats_pkl_file)
        topModes = pickle.load(modes_pkl_file)
        currNModes = len(topModes)
        if currNModes == 0:
            break

        if prevState != None:
            angChange = gtState['gps_heading'] - prevState['gps_heading']
            while angChange < -np.pi:
                angChange += 2.0*np.pi
            while angChange > np.pi:
                angChange += -2.0*np.pi
            distChange = np.sqrt(np.sum(np.power(gtState['gps_position'].reshape(2) - prevState['gps_position'].reshape(2),2.0)))
        else:
            angChange = 0
            distChange = 0
        moveDists.append((angChange,distChange))

        dataTimes.append(t)
        modes.append(topModes)
        cgpsErr = np.sqrt(np.sum(np.power(gtState['gps_position'].reshape(2) - gtState['map_position'].reshape(2),2.0)))
        errors['gpsErr'].append(cgpsErr) 

        currErrs = []
        currMapErrs = []
        for (p,curr_street,curr_pos,curr_ang) in topModes:
            angErr = curr_ang - gtState['gps_heading']
            while angErr < -np.pi:
                angErr += 2.0*np.pi
            while angErr > np.pi:
                angErr += -2.0*np.pi
            currErrs.append((np.sqrt(np.sum(np.power(curr_pos.reshape(2) - gtState['gps_position'].reshape(2),2.0))), np.abs(angErr)))
            currMapErrs.append((np.sqrt(np.sum(np.power(curr_pos.reshape(2) - gtState['map_position'].reshape(2),2.0))), np.abs(angErr)))

        if currNModes == 1 and currErrs[0][0] < localizationPosErrThresh and currErrs[0][1] < localizationAngErrThresh:
            seqLocalFrames += 1
        else:
            seqLocalFrames = 0
        
        if not localized and seqLocalFrames >= localizationSeqFramesThresh:
            localizedInd = frameCount - localizationSeqFramesThresh
            localized = True
            for locI in range(localizationSeqFramesThresh-1):
                localizedFlags[-(1+locI)] = True
        localizedFlags.append(localized)

        currErrs = np.array(currErrs).reshape(-1,2)
        currMapErrs = np.array(currMapErrs).reshape(-1,2)
        
        for (i,cN) in enumerate(topErrN):
            ccN = np.minimum(cN,currNModes)
            errors['absGPSErr'][i].append(np.array([np.min(currErrs[0:ccN,0]), \
                                                    np.min(currErrs[0:ccN,1])]))
            errors['absMapErr'][i].append(np.array([np.min(currMapErrs[0:ccN,0]), \
                                                    np.min(currMapErrs[0:ccN,1])]))
            errors['relErr'][i].append(np.min(currErrs[0:ccN,0])/cgpsErr)
            
            if not options.mpi:
                print '{0} best error = {1}m {2} deg'.format(cN,topErrs[i][-1][0]*1000.0,topErrs[i][-1][1]*(180.0/np.pi))

        prevState = gtState

    if not localized:
        localizedInd = frameCount+1
        localizedTime = np.nan
    else:
        localizedTime = dataTimes[localizedInd]
 
    modes_pkl_file.close()
    stats_pkl_file.close()

    # Open output file
    pickle.dump({'topErrN':topErrN,'modes':modes,'errors':errors, \
                 'localizedFlags':localizedFlags,'localizedInd':localizedInd, \
                 'localizedTime':localizedTime,'dataTimes':dataTimes, \
                 'moveDists':moveDists}, \
                open(get_outfile_path(options,'errors.p'),'wb'))
    
    return (modes,errors,localizedFlags,localizedInd,localizedTime)

def do_load_errors(options,args):
    load_config_file(options,args)

    assert options.gps_data != None, 'GPS data is required to compute errors'

    errorInfo = pickle.load(open(get_outfile_path(options,'errors.p'),'rb'))
    topErrN = errorInfo['topErrN'] 
    errors = errorInfo['errors']
    localizedInd = errorInfo['localizedInd']
    modes = errorInfo['modes']
    localizedFlags = errorInfo['localizedFlags']
    localizedTime = errorInfo['localizedTime']
    moveDists = errorInfo['moveDists']

    return (modes,errors,localizedFlags,localizedInd,localizedTime,topErrN,moveDists)

def do_latex_tables(options,args):
    dynNameMap = {'state2rigid':'Highway \& Road Parameters', 'state2rigid_1kind':'Single Parameter'}
    sourceNameMap = {'Stereo':'S','Monocular':'M','GPS':'G','Oracle':'O'}
    sourceOrdering = {'Stereo':2,'Monocular':1,'GPS':3}
    
    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    outFile = args[0]
    assert outFile.endswith('.tex'),'First argument must be a tex filename to output table'
    args = args[1:]

    errorInfo = {}
    sequences = set()
    sources = set()
    motionModels = set()
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        cOptions = copy(options)
        load_config_file(cOptions,args[i])

        sequences.add(cOptions.sequence_id)
        sources.add(cOptions.data_source)
        motionModels.add(cOptions.dynamics)

        key = (cOptions.sequence_id,cOptions.data_source,cOptions.dynamics)
        assert key not in errorInfo,'Repeated combination of sequence, source and motion model detected'
        errorInfo[key] = pickle.load(open(get_outfile_path(cOptions,'errors.p'),'rb'))

    if options.mpi:
        allModeInfo = comm.gather((errorInfo,sequences,sources,motionModels))
        if rank == 0:
            for cErrInfo in allModeInfo:
                errorInfo.update(cErrInfo[0])
                sequences |= cErrInfo[1]
                sources |= cErrInfo[2]
                motionModels |= cErrInfo[3]
    
    if rank != 0:
        return

    sequences = sorted(sequences)
    sources = sorted(sources, key=lambda src: sourceOrdering[src])
    motionModels = sorted(motionModels)

    errKeys = None
    tblStats = {'average':{},'sum':{}}
    seqStats = {}
    for seqId in sequences:
        seqStats[seqId] = {}
        initSeqStats = False
        tblStats[seqId] = {'average':{},'sum':{}}
        for sourceId in sources:
            tblStats[seqId][sourceId] = {'average':{},'sum':{}}
            for mmName in motionModels:

                key = (seqId,sourceId,mmName)
                assert key in errorInfo,'Missing combination: {0}'.format(key)

                tblStats[seqId][sourceId][mmName] = {}

                cErrInfo = errorInfo[key]['errors']
                cLocFlags = np.array(errorInfo[key]['localizedFlags'])
                cLocTime = errorInfo[key]['localizedTime']
                cLocInd = errorInfo[key]['localizedInd']
                cMoveDists = np.array(errorInfo[key]['moveDists'])
                
                if not initSeqStats:
                    initSeqStats = True
                    seqStats[seqId]['mapErr'] = np.mean(np.array(errorInfo[key]['errors']['gpsErr']))
                
                currAbsErrs = np.array(cErrInfo['absGPSErr'])
                tblStats[seqId][sourceId][mmName]['map_error_mean'] = np.mean(currAbsErrs[0,:,:],0)
                tblStats[seqId][sourceId][mmName]['map_localize_error_mean'] = np.mean(currAbsErrs[0,cLocFlags,:],0)
                tblStats[seqId][sourceId][mmName]['map_error_sum'] = np.sum(currAbsErrs[0,:,:],0)
                tblStats[seqId][sourceId][mmName]['map_localize_error_sum'] = np.sum(currAbsErrs[0,cLocFlags,:],0)
                tblStats[seqId][sourceId][mmName]['nframes'] = currAbsErrs.shape[1]
                tblStats[seqId][sourceId][mmName]['nframes_localize'] = np.sum(cLocFlags)
                tblStats[seqId][sourceId][mmName]['localized'] = np.isfinite(cLocTime)
                if tblStats[seqId][sourceId][mmName]['localized']:
                    tblStats[seqId][sourceId][mmName]['not_nan_localize_time'] = cLocTime
                else:
                    tblStats[seqId][sourceId][mmName]['not_nan_localize_time'] = 0
                tblStats[seqId][sourceId][mmName]['localize_time'] = cLocTime
                tblStats[seqId][sourceId][mmName]['localize_dist'] = np.sum(cMoveDists[0:cLocInd,:],0)
                
                if errKeys == None:
                    errKeys = tblStats[seqId][sourceId][mmName].keys()
        
    for sourceId in sources:
        tblStats['average'][sourceId] = {}
        tblStats['sum'][sourceId] = {}
        for mmName in motionModels:
            tblStats['average'][sourceId][mmName] = {} 
            tblStats['sum'][sourceId][mmName] = {} 
            for errKey in errKeys:
                tblStats['average'][sourceId][mmName][errKey] = 0 
                tblStats['sum'][sourceId][mmName][errKey] = 0 
                for seqId in sequences:
                    tblStats['average'][sourceId][mmName][errKey] += tblStats[seqId][sourceId][mmName][errKey]/float(len(sequences)) 
                    tblStats['sum'][sourceId][mmName][errKey] += tblStats[seqId][sourceId][mmName][errKey] 

    seqErrAvg = 0
    for seqId in sequences:
        seqErrAvg += seqStats[seqId]['mapErr']/len(sequences)

    fid = open(outFile,'wt')
#    # Vertical layout
#    fid.write('\\begin{table*}\n')
#    fid.write('\\centering\\small\n')
#    fid.write('\\begin{tabular}{cc|c|' + '|ccc' * len(motionModels) + '|}\n')
#    fid.write('         &        & ')
#    for mmName in motionModels: fid.write('& \multicolumn{3}{|c|}{' + dynNameMap[mmName] + '}')
#    fid.write(' \\\\\n')
#    fid.write(' & & ' + ' & \multicolumn{2}{|c}{MAP Error} & Localization ' * len(motionModels) + '\\\\\n')
#    fid.write(' & & Map Error ' + ' & Position & Heading & Time ' * len(motionModels) + '\\\\\n')
#    fid.write('\\hline\n'*2)
#    for seqId in sequences:
#        for (srcN,sourceId) in enumerate(sources):
#            if srcN == 0:
#                rowString = '\\multirow{' + '{0}'.format(len(sources)) + '}}{{*}}{{{0}}} '.format(seqId)
#            else:
#                rowString = ' '
#            rowString += '& {0} '.format(sourceNameMap[sourceId])
#            if srcN == 0:
#                rowString += '& \\multirow{' + '{0}'.format(len(sources)) + '}}{{*}}{{{0:.2f}m}} '.format(1000.0*seqStats[seqId]['mapErr'])
#            else:
#                rowString += '& '
#                
#            for mmName in motionModels:
#                cErr = tblStats[seqId][sourceId][mmName]['map_localize_error_mean']
#                cTime = tblStats[seqId][sourceId][mmName]['localize_time']
#                if np.isfinite(cTime):
#                    rowString += '& {0:.2f}m & {1:.2f}$^\\circ$ & {2:.0f}s'.format(1000.0*cErr[0],(180.0/np.pi)*cErr[1],cTime)
#                else:
##                    rowString += '& \multicolumn{3}{|c|}{Unable to Localize}'
#                    rowString += '& *' * 3
#            
#            rowString += '\\\\\n'
#            fid.write(rowString)
#        fid.write('\\hline\n')
#    fid.write('\\hline\n')
#    seqId = 'average'
#    for (srcN,sourceId) in enumerate(sources):
#        if srcN == 0:
#            rowString = '\\multirow{' + '{0}'.format(len(sources)) + '}{*}{Mean}'
#        else:
#            rowString = ''
#
#        rowString += ' & {0} '.format(sourceNameMap[sourceId])
#        if srcN == 0:
#            rowString += '& \\multirow{' + '{0}'.format(len(sources)) + '}}{{*}}{{{0:.2f}m}} '.format(1000.0*seqErrAvg)
#        else:
#            rowString += '& '
#
#        for mmName in motionModels:
#            cErr = tblStats['sum'][sourceId][mmName]['map_localize_error_sum'] / tblStats['sum'][sourceId][mmName]['nframes_localize'] 
#            cTime = tblStats['sum'][sourceId][mmName]['not_nan_localize_time'] / tblStats['sum'][sourceId][mmName]['localized']
#            rowString += ' & {0:.2f}m & {1:.2f}$^\\circ$ & {2:.2f}s'.format(1000.0*cErr[0],(180.0/np.pi)*cErr[1],cTime)
#        
#        rowString += ' \\\\\n'
#        fid.write(rowString)
#    fid.write('\\hline\n')
#    fid.write('\\end{tabular}\n')
#    fid.write('\\end{table*}\n\n')

    # Horizontal layout
#    fid.write('\\begin{table*}\n')
#    fid.write('\\centering\\small\n')
    fid.write('\\begin{tabular}{cc|' + '|c' * len(sequences) + '||c|}\n')
    fid.write(' & ')
    for seqId in sequences + ['Average']: fid.write(' & ' + seqId + ' ')
    fid.write(' \\\\\n')
    
#    rowString = ' \\multicolumn{2}{c||}{Map Error} '
#    for seqId in sequences:
#        rowString += ' & {0:.2f}m '.format(1000.0*seqStats[seqId]['mapErr'])
#    rowString += ' & {0:.2f}m \\\\\n'.format(1000.0*seqErrAvg)
#    fid.write(rowString)

    for (mmI,mmName) in enumerate(motionModels):
        fid.write('\\hline\n'*2)
        if len(motionModels) > 1:
            fid.write(' & & \multicolumn{{{0}}}{{|c|}}{{{1}}} '.format(len(sequences)+1, dynNameMap[mmName]))
            fid.write(' \\\\\n')
            fid.write('\\hline\n'*2)

        cSources = sources + ['Oracle']
        for (srcI,sourceId) in enumerate(cSources):
            if srcI == 0:
                rowString = ' \\multirow{{{0}}}{{*}}{{Position Error}} '.format(len(cSources))
            else:
                rowString = ' '
            rowString += ' & ' + sourceNameMap[sourceId] + ' '

            if srcI < len(sources):
                for seqId in sequences:
                    cErr = tblStats[seqId][sourceId][mmName]['map_localize_error_mean']
                    if np.isfinite(cErr[0]):
                        rowString += '& {0:.1f}m '.format(1000.0*cErr[0])
                    else:
                        rowString += '& * '
                cErr = tblStats['sum'][sourceId][mmName]['map_localize_error_sum'] / tblStats['sum'][sourceId][mmName]['nframes_localize'] 
                rowString += '& {0:.1f}m '.format(1000.0*cErr[0])
            else:
                for seqId in sequences:
                    rowString += '& {0:.1f}m '.format(1000.0*seqStats[seqId]['mapErr'])
                rowString += ' & {0:.2f}m '.format(1000.0*seqErrAvg)

            rowString += ' \\\\\n'
            fid.write(rowString)

        fid.write('\\hline\n')

        for (srcI,sourceId) in enumerate(sources):
            if srcI == 0:
                rowString = ' \\multirow{{{0}}}{{*}}{{Heading Error}} '.format(len(sources))
            else:
                rowString = ' '
            rowString += ' & ' + sourceNameMap[sourceId] + ' '

            for seqId in sequences:
                cErr = tblStats[seqId][sourceId][mmName]['map_localize_error_mean']
                if np.isfinite(cErr[1]):
                    rowString += '& {0:.1f}$^\\circ$ '.format((180.0/np.pi)*cErr[1])
                else:
                    rowString += '& * '

            cErr = tblStats['sum'][sourceId][mmName]['map_localize_error_sum'] / tblStats['sum'][sourceId][mmName]['nframes_localize'] 
            rowString += '& {0:.1f}$^\\circ$ '.format((180.0/np.pi)*cErr[1])
            rowString += ' \\\\\n'
            fid.write(rowString)

#        fid.write('\\hline\n')
#
#        for (srcI,sourceId) in enumerate(sources):
#            if srcI == 0:
#                rowString = ' \\multirow{{{0}}}{{*}}{{Localization Time}} '.format(len(sources))
#            else:
#                rowString = ' '
#            rowString += ' & ' + sourceNameMap[sourceId] + ' '
#            for seqId in sequences:
#                cTime = tblStats[seqId][sourceId][mmName]['localize_time']
#                if np.isfinite(cTime):
#                    rowString += '& {0:.0f}s '.format(cTime)
#                else:
#                    rowString += '& * '
#
#            cTime = tblStats['sum'][sourceId][mmName]['not_nan_localize_time'] / tblStats['sum'][sourceId][mmName]['localized']
#            rowString += '& {0:.2f}s '.format(cTime)
#            rowString += ' \\\\\n'
#            fid.write(rowString)
    fid.write('\\hline\n')
    fid.write('\\end{tabular}\n')
#    fid.write('\\end{table*}\n\n')
    fid.close()


def plot_frame_errors(options,args,displayErrN):
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    load_config_file(options,args)

    assert options.gps_data != None, 'GPS data is required to compute errors'

    errorInfo = pickle.load(open(get_outfile_path(options,'errors.p'),'rb'))
    topErrN = errorInfo['topErrN'] 
    errors = errorInfo['errors']
    topErrs = errors['absGPSErr']
    localizedInd = errorInfo['localizedInd']
    modes = errorInfo['modes']
    localizedFlags = errorInfo['localizedFlags']
    localizedTime = errorInfo['localizedTime']
    dataTimes = errorInfo['dataTimes']

    fig = plt.figure(figsize=(plotFigSize[0],2*plotFigSize[1]),dpi=plotDPI)

    fig.clf()
    ax = fig.add_subplot(2,1,1)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes,1000.0*cErrs[:,0], label = clabel)
    ax.set_ylabel('Error (meters)')
    ax.set_title('Positional Error')
    ax.legend()

    ax = fig.add_subplot(2,1,2)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes,(180.0/np.pi)*cErrs[:,1], label = clabel)
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Error (degrees)')
    ax.set_title('Heading Error')
    ax.legend()

    fname = get_outfile(options,'error.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)
    
    fig.clf()
    ax = fig.add_subplot(2,1,1)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes,1000.0*cErrs[:,0], label = clabel)
    ax.set_ylabel('Error (meters)')
    ax.set_title('Positional Error')
    ax.set_yscale('log')
    ax.legend()
    
    ax = fig.add_subplot(2,1,2)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes,(180.0/np.pi)*cErrs[:,1], label = clabel)
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Error (degrees)')
    ax.set_title('Heading Error')
    ax.set_yscale('log')
    ax.legend()
    
    fname = get_outfile(options,'logerror.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)

    ### Now output the localized error plots ###
    fig.clf()
    ax = fig.add_subplot(2,1,1)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes[localizedInd:],1000.0*cErrs[localizedInd:,0], label = clabel)
    ax.set_ylabel('Error (meters)')
    ax.set_title('Positional Error')
    ax.legend()

    ax = fig.add_subplot(2,1,2)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes[localizedInd:],(180.0/np.pi)*cErrs[localizedInd:,1], label = clabel)
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Error (degrees)')
    ax.set_title('Heading Error')
    ax.legend()

    fname = get_outfile(options,'localized-error.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)
    
    fig.clf()
    ax = fig.add_subplot(2,1,1)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes[localizedInd:],1000.0*cErrs[localizedInd:,0], label = clabel)
    ax.set_ylabel('Error (meters)')
    ax.set_title('Positional Error')
    ax.set_yscale('log')
    ax.legend()
    
    ax = fig.add_subplot(2,1,2)
    for (i,(cN,cErrList)) in enumerate(izip(topErrN,topErrs)):
        if cN not in displayErrN: continue
        cErrs = np.array(cErrList)
        if cN == 1:
            clabel = 'MAP Error'
        else:
            clabel = '{0} best error'.format(cN)
        ax.plot(dataTimes[localizedInd:],(180.0/np.pi)*cErrs[localizedInd:,1], label = clabel)
    ax.set_xlabel('Data time (seconds)')
    ax.set_ylabel('Error (degrees)')
    ax.set_title('Heading Error')
    ax.set_yscale('log')
    ax.legend()
    
    fname = get_outfile(options,'localized-logerror.{0}'.format(options.img_format))
    fig.savefig(fname,dpi=plotDPI)
    
    return (modes,errors,localizedFlags,localizedInd,localizedTime,topErrN)

def do_plot_errors(options,args):
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0
 
    if args[0].endswith('.dcfg'):
        srcName = 'default'
    else:
        srcName = args[0]
        args = args[1:]

    modeInfo = {}
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        modeInfo[args[i]] = plot_frame_errors(copy(options),args[i],displayErrNs)

    if options.mpi:
        allModeInfo = comm.gather(modeInfo)
        if rank == 0:
            for cmode in allModeInfo:
                modeInfo.update(cmode)
    
    if rank == 0:
        seqErrs = np.empty((len(topErrNs),len(args)+1,2,2))
        gpsErrs = np.empty((len(args)+1,))
        nFrames = np.empty((len(args),))
        localizeTimes = np.empty((len(args)+1,))
        localizedFlags = []
        totalLocalizedErrs = 0
        totalLocalizedFrames = 0
        totalErrs = 0
        totalFrames = 0
        for (cfgI,cfgFile) in enumerate(args):
            if modeInfo[cfgFile][1] == None:
                print 'No GPS data for {0}.'.format(cfgFile)
                continue
            print '{0}:'.format(cfgFile)
            currErrs = np.array(modeInfo[cfgFile][1]['absGPSErr'])
            currGPSErrs = np.array(modeInfo[cfgFile][1]['gpsErr'])
            localizedFrameFlags = np.array(modeInfo[cfgFile][2])
            for (i,cN) in enumerate(topErrNs):
                posErr,angErr = 1000.0*np.mean(currErrs[i,:,0]), (180.0/np.pi)*np.mean(currErrs[i,:,1])
                posLocalErr,angLocalErr = 1000.0*np.mean(currErrs[i,localizedFrameFlags,0]), (180.0/np.pi)*np.mean(currErrs[i,localizedFrameFlags,1])
                seqErrs[i,cfgI,0,0] = posErr
                seqErrs[i,cfgI,1,0] = angErr
                seqErrs[i,cfgI,0,1] = posLocalErr
                seqErrs[i,cfgI,1,1] = angLocalErr
            totalErrs += np.sum(currErrs,1)
            totalFrames += currErrs.shape[1]
            totalLocalizedErrs += np.sum(currErrs[:,localizedFrameFlags,:],1)
            totalLocalizedFrames += np.sum(localizedFrameFlags)
            localizeTimes[cfgI] = modeInfo[cfgFile][4]
            localizedFlags.append(np.isfinite(modeInfo[cfgFile][4]))
            gpsErrs[cfgI] = 1000.0*np.mean(currGPSErrs)
            nFrames[cfgI] = currGPSErrs.shape[0]

        localizedFlags.append(False)
        localizedFlags = np.array(localizedFlags)
        localizeTimes[len(args)] = np.mean(localizeTimes[localizedFlags])
        
        gpsErrs[len(args)] = np.sum(np.multiply(nFrames/np.sum(nFrames),gpsErrs[0:len(args)]))

        for (i,cN) in enumerate(topErrNs):
            posErr,angErr = 1000.0*(totalErrs[i,0]/totalFrames), (180.0/np.pi)*(totalErrs[i,1]/totalFrames)
            posLocalErr,angLocalErr = 1000.0*(totalLocalizedErrs[i,0]/totalLocalizedFrames), (180.0/np.pi)*(totalLocalizedErrs[i,1]/totalLocalizedFrames)
            seqErrs[i,len(args),0,0] = posErr
            seqErrs[i,len(args),1,0] = angErr
            seqErrs[i,len(args),0,1] = posLocalErr
            seqErrs[i,len(args),1,1] = angLocalErr

        rowLabels = []
        for cfgFile in args:
            rowLabels.append(os.path.basename(cfgFile).split('.')[0])
        rowLabels.append('Average')
        csvColLabels = []
        csvColLabels.append('Time to Localize (s)')
        csvColLabels.append('GPS Error (m)')
        for (i,cN) in enumerate(topErrNs):
            if cN == 1:
                errName = 'MAP'
            else:
                errName = 'Top {0}'.format(cN)
            csvColLabels.append(errName + ' Position Error (m)')
            csvColLabels.append(errName + ' Heading Error (deg)')

        locErrTypes = ['Overall','Localized']
        for (locTypeI,locType) in enumerate(locErrTypes):
            csvFID = open(os.path.join(options.out_dir,'{1}-{0}-ErrorTable.csv'.format(locType,srcName)),'wt')
            csvWriter = csv.writer(csvFID)
            csvWriter.writerow([''] + csvColLabels)
            for i in range(seqErrs.shape[1]):
                csvWriter.writerow( [rowLabels[i]]  + [localizeTimes[i]] + [e for e in seqErrs[:,i,:,locTypeI].reshape(-1)])
            csvFID.close()

        dispColLabels = []
        for (i,cN) in enumerate(displayErrNs):
            if cN == 1:
                errName = 'MAP'
            else:
                errName = 'Top {0}'.format(cN)
            dispColLabels.append(errName)
        dispColLabels.append('Time to Localize (s)')

        fig = plt.figure(figsize=(11,8.5),dpi=plotDPI)
        errTypes = [('Position','meters','m'),('Heading','degrees', 'deg')];
        for (locTypeI,locType) in enumerate(locErrTypes):
            fig.clf()
            for (errTypeI,errType) in enumerate(errTypes):
                cellText = []
                for i in range(seqErrs.shape[1]):
                    cellText.append(['{0:.2f}'.format(e) for (e,cN) in izip(seqErrs[:,i,errTypeI,locTypeI],topErrNs) if cN in displayErrNs ] + ['{0:.2f}'.format(localizeTimes[i])])
    
                ax = fig.add_subplot(2,1,errTypeI+1)
                ax.table(cellText = cellText, colLabels = dispColLabels, rowLabels = rowLabels, loc = 'center right')
                ax.axis('off')
                ax.set_title('{3} {2} {0} Error ({1})'.format(errType[0],errType[1],srcName,locType))
    
            fname = os.path.join(options.out_dir,'{1}-{0}ErrorTable.pdf'.format(locType,srcName))
            fig.savefig(fname,dpi=plotDPI)

            fig.clf()
            for (errTypeI,errType) in enumerate(errTypes):
                ax = fig.add_subplot(2,1,errTypeI+1)
                ax.plot(topErrNs,seqErrs[:,-1,errTypeI,locTypeI])
                ax.set_ylabel('Best Error ({0})'.format(errType[1]))
                ax.set_xlabel('Number of Modes')
                ax.set_xlim([np.min(plotErrNs),np.max(plotErrNs)])
                ax.set_title('{3} {2} {0} Error'.format(errType[0],errType[1],srcName,locType))
            fname = os.path.join(options.out_dir,'{1}-{0}AverageErrorPlot.{2}'.format(locType,srcName,options.img_format))
            fig.savefig(fname,dpi=plotDPI)

def do_extract_modes(options,args):
    nmsNumModes = 200 # Maximum number of modes to return (may actually return fewer)
    nmsSegmentLength = 5.0/1000.0 # Segment length to integrate the CDF over for finding initial mode candidates
    nmsRadius = 30.0/1000.0 # Any smaller modes within this range are suppressed
    nmsCutoffPct = 0.01 # All modes must be within this pct of the top mode

    nmsParams = {'nmsNumModes':nmsNumModes, 'nmsSegmentLength':nmsSegmentLength, 'nmsRadius':nmsRadius, 'nmsCutoffPct':nmsCutoffPct}
    
    load_config_file(options,args)

    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    # Load the mapgraph
    mapgraph,mapVersion = load_mapgraph(options)

    # Get data file and statistics
    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    mapdynamics = baseData['mapdynamics']
    start_frame = baseData['start_frame']
    stats_pkl_file = open(get_outfile_path(options,'stats.p'),'rb')

    # Open the file with the posteriors
    if baseData['mpi']:
        post_pkl_files = [ open(get_outfile_path(options,'posteriors-rank{0:03}.p'.format(crank)),'rb') for crank in range(baseData['size']) ]
        post = None
        for pfile in post_pkl_files:
            if post == None:
                post = pickle.load(pfile)
            else:
                post.update(pickle.load(pfile))
    else:
        post_pkl_file = open(get_outfile_path(options,'posteriors.p'),'rb')
        post = pickle.load(post_pkl_file)

    # Open output file
    modes_pkl_file = open(get_outfile_path(options,'modes.p'),'wb')
    pickle.dump(nmsParams,modes_pkl_file)

    firstReadDone = False
    for curr_obs in baseData['obs']:
        t = curr_obs['t']
        tInd = curr_obs['tInd']
        curr_data_dt = curr_obs['dt']
        yt = curr_obs['obs']
        
        if tInd < start_frame:
            continue

        try:
            stepData = pickle.load(stats_pkl_file)
        except:
            print 'Failed to load step data at tInd = {0} in file {1}'.format(tInd,get_outfile_path(options,'stats.p'))

        if not options.mpi:
            print '\n\ntInd = {0}, t = {2}, obst = {1}'.format(tInd,yt.T,t)

        # Read the posterior
        if baseData['mpi']:
            for (frameRef,pfile) in izip(stepData['post_pos_ref'],post_pkl_files):
                if not firstReadDone:
                    pfile.seek(frameRef)
                post.update(pickle.load(pfile))
        else:
            if not firstReadDone:
                post_pkl_file.seek(stepData['post_pos_ref'][0])
            post = pickle.load(post_pkl_file)
        firstReadDone = True

        # Compute the modes
        topModes = post.roadSegmentModes(mapgraph,mapdynamics,nmsNumModes,nmsSegmentLength,nmsRadius,nmsCutoffPct)
        
        # Dump the modes
        pickle.dump(topModes,modes_pkl_file)

    modes_pkl_file.close()
    stats_pkl_file.close()
    if baseData['mpi']:
        for pfile in post_pkl_files:
            pfile.close()
    else:
        post_pkl_file.close()

def do_display_gt(options,args):
    assert(len(args) == 1)
    load_config_file(options,args[0])

    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    mapgraph,mapVersion = load_mapgraph(options)
    mapSz = mapgraph.posMax - mapgraph.posMin

    dpi = 100
    
    sortedStreets = mapgraph.nodes()
    sortedStreets.sort()

    assert(options.gps_data != None)
    if options.gps_data != None:
        stateIter = open(options.gps_data,'rt')
        
    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']
    latlon = np.empty(2)
    fig = plt.figure()
    fig.set_dpi(dpi)
    fig.set_size_inches(10,10)
    ax = fig.add_subplot(111)
    
    for (tInd,statet) in enumerate(stateIter):
        lineParts = statet.split()
    
        latlon[0] = float(lineParts[0])
        latlon[1] = float(lineParts[1])
        gtPos = mercatorProj(latlon,projScale) - ptCoordOrigin

        fig.clf()
        ax = fig.add_subplot(111)
        ax.set_xlim(gtPos[0] - 0.125,gtPos[0] + 0.125)
        ax.set_ylim(gtPos[1] - 0.125,gtPos[1] + 0.125)
        mapgraph.display(fig, ax, gtPos = gtPos)
        fname = get_outfile_path(options,'gtDisplay{0:04}.{1}'.format(tInd+1,options.img_format))
        extent = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
        fig.savefig(fname,bbox_inches=extent.expanded(1.1, 1.1),dpi=dpi)
#            fig.savefig(fname,dpi=dpi)

def do_plot_data(options):
    import matplotlib.pyplot as plt

    baseData = pickle.load(open(get_datafile_path(options),'rb'))
    states = baseData['states']
    mapdynamics = baseData['mapdynamics']
    Astate_vel = mapdynamics.get_state_vel_matrix()

    time_mat = np.zeros((len(states)))
    states_mat = np.zeros((len(states),4))
    streets_mat = np.zeros((len(states)))
    vtranslp_mat = np.zeros((len(states)-1))
    prevStreet = None
    for i in range(len(states)):
        states_mat[i,:] = np.dot(Astate_vel,states[i]['state']).T
        streets_mat[i] = states[i]['street'] == prevStreet
        time_mat[i] = states[i]['time']
        prevStreet = states[i]['street']
        if i > 0:
            vtranslp_mat[i-1] = mapdynamics.street_transition_logprob(mapgraph,prevStreet,states[i-1]['street'],states[i-1]['state'])
    
    labels = ('$d$','$\dot{d}$',r'$\theta$',r'$\dot{\theta}$')
    scales = (1,1,180.0/np.pi,180.0/np.pi)
    plt.figure(1)
    for i in range(4):
        plt.subplot(2,2,i+1)
        plt.plot(time_mat,scales[i]*states_mat[:,i])
        plt.title(labels[i])

    plt.figure(2)
    plt.subplot(2,1,1)
    plt.plot(time_mat,streets_mat,'--')
    
    plt.subplot(2,1,2)
    plt.plot(time_mat[1:],np.exp(vtranslp_mat))
    
    plt.show()

def convert_gps_trajectory(mapgraph,mapdynamics,statefile,dt,data_skip):
    debugPrint = False
    roadPosThresh = 1.0/1000.0
    nbhdThresh = 10.0/1000.0
    dirThresh = np.pi/2.0
    transAngleThresh = np.pi
    transDistThresh = 30.0/1000.0
    errAngW = 1.0/(np.pi/8.0)
    errDistW = 1.0

    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']

    latlon = np.empty(2)
    instVel = np.empty(2)
    avgInstVel = np.zeros(2)

    pos = None

    roadSetSeq = {}
    posInfos = {}
    
    #print 'Parsing GPS data and finding nearest streets...',
    stateFile = open(statefile,'rt')
    for (tInd,line) in enumerate(stateFile):
        lineParts = line.split()

        latlon[0] = float(lineParts[0])
        latlon[1] = float(lineParts[1])
        instVF = float(lineParts[9])/1000.0
        instVel[0] = float(lineParts[7])/1000.0 
        instVel[1] = float(lineParts[6])/1000.0 
        
        pos = mercatorProj(latlon,projScale) - ptCoordOrigin
        heading = float(lineParts[5])
        headingDir = np.array([np.cos(heading),np.sin(heading)])

        if tInd == 0:
            currRoads = None
            nbhdRdSet = None
        else:
            currRoads = roadSetSeq[tInd-1]
            nbhdRdSet = set()
            for prevStreet in currRoads.iterkeys():
                for nextStreet in mapgraph.successors_iter(prevStreet):
#                    if nextStreet == SINK_ROAD or np.abs(mapgraph[prevStreet][nextStreet]['transition_angle']) > transAngleThresh or mapgraph[prevStreet][nextStreet]['transition_distance'] > mapgraph.node[prevStreet]['length'] + transDistThresh:
                    if nextStreet == SINK_ROAD or (mapgraph[prevStreet][nextStreet]['transition_distance'] - mapgraph.node[prevStreet]['length']) > transDistThresh:
#                    if nextStreet == SINK_ROAD:
                        continue
                    nbhdRdSet.add(nextStreet)

            if len(nbhdRdSet) == 0:
                print '\nERROR: no roads found in neighbourhood set at tInd = {0}'.format(tInd)
                print '  Transition Angle Threshold: {0}'.format((180.0/np.pi)*transAngleThresh)
                print 'Previous Streets:'
                for prevStreet in currRoads.iterkeys():
                    print '{0}:'.format(prevStreet)
                    for nextStreet in mapgraph.successors_iter(prevStreet):
                        if nextStreet == SINK_ROAD:
                            continue
                        print ' -> {0}:\n    transAngle = {1}, transDistExcess = {2} '.format(nextStreet,(180.0/np.pi)*mapgraph[prevStreet][nextStreet]['transition_angle'],mapgraph[prevStreet][nextStreet]['transition_distance'] - mapgraph.node[prevStreet]['length'])
                assert(len(nbhdRdSet) > 0)

        (nbhdRds,distDataFoo) = mapgraph.getNearbyRoads(pos,headingDir,nbhdThresh,dirThresh,roadPosThresh,nbhdRdSet)

        if debugPrint:
            print '\n\ntInd = {0}'.format(tInd)
            print '  Distance Threshold: {0}, Direction Threshold: {1}, Alpha Threshold: {2}'.format(nbhdThresh,(180.0/np.pi)*dirThresh,roadPosThresh)

        if len(nbhdRds) == 0:
            print '\nERROR: no nearby roads found in neighbourhood set at tInd = {0}'.format(tInd)

        if (debugPrint or len(nbhdRds) == 0) and currRoads != None:
            print '\nCandidate Roads at previous frame:'
            for (prevRd,distInfo) in currRoads.iteritems():
                print '    {0}: dist = {1}, relAngle = {2}, alpha = {3} (len = {4})'.format(prevRd,distInfo[0],(180.0/np.pi)*distInfo[1],distInfo[3],mapgraph.node[prevRd]['length'])
                mapgraph.printNode(prevRd)
            print '\nNeighbourhood roads:'
            for nbRd in nbhdRdSet:
                distInfo = mapgraph.distanceToRoad(nbRd,pos,headingDir)
                print '    {0}: dist = {1}, relAngle = {2}, alpha = {3} (len = {4})'.format(nbRd,distInfo[0],(180.0/np.pi)*distInfo[1],distInfo[3],mapgraph.node[nbRd]['length'])

        if debugPrint:
            print '\nCurrent candidate roads:'
            for (prevRd,distInfo) in nbhdRds.iteritems():
                print '    {0}: dist = {1}, relAngle = {2}, alpha = {3} (len = {4})'.format(prevRd,distInfo[0],(180.0/np.pi)*distInfo[1],distInfo[3],mapgraph.node[prevRd]['length'])

        assert(len(nbhdRds) > 0)
        roadSetSeq[tInd] = deepcopy(nbhdRds)
        posInfos[tInd] = (pos,np.copy(latlon),np.copy(instVF),np.copy(instVel),heading)
        
        # Prune roads at previous time steps which don't connect to the current set of road possibilities
        for ctInd in range(tInd,0,-1):
            if len(roadSetSeq[ctInd-1]) == 1:
                break

            rmRoads = set()
            for prevStreet in roadSetSeq[ctInd-1].iterkeys():
                connected = False
                for nextStreet in roadSetSeq[ctInd].iterkeys():
                    if mapgraph.has_edge(prevStreet,nextStreet) and (mapgraph[prevStreet][nextStreet]['transition_distance'] - mapgraph.node[prevStreet]['length']) <= transDistThresh:
                        connected = True
                        break
                if not connected:
                    rmRoads.add(prevStreet)
                
#            print 'back pruning {1} at {0}'.format(ctInd-1,len(rmRoads))

            for delStreet in rmRoads:
                del (roadSetSeq[ctInd-1])[delStreet]
                    
            if len(rmRoads) == 0:
                break                        
        
    stateFile.close()
    #print 'done.'

    #print 'Disambiguating nearby roads...',
    while True:
        # Find least ambiguous street.
        minErr = np.inf
        minErrInd = None
        minErrRoad = None
        for (tInd,currRoads) in roadSetSeq.iteritems():
            if len(currRoads) == 1:
                continue
            for (currStreet,distInfo) in currRoads.iteritems():
                currErr = errDistW*distInfo[0] + errAngW*np.abs(distInfo[1])
                if currErr < minErr:
                    minErr = currErr
                    minErrInd = tInd
                    minErrRoad = currStreet
                    minErrDistInfo = distInfo
                    
        # No more ambiguous roads.
        if minErrInd == None:
            break
        
        # Prune other possibilities
        #print 'Selecting {0} at {1}'.format(minErrRoad,minErrInd)
        roadSetSeq[minErrInd] = {minErrRoad:minErrDistInfo}

        # Prune forwards
        for tInd in range(minErrInd,len(roadSetSeq)-1,1):
            prevRoads = roadSetSeq[tInd]
            nextRoads = roadSetSeq[tInd+1]
            if len(nextRoads) == 1:
                break

            rmRoads = set()
            for nextStreet in nextRoads.iterkeys():
                connected = False
                for prevStreet in prevRoads.iterkeys():
                    if mapgraph.has_edge(prevStreet,nextStreet) and (mapgraph[prevStreet][nextStreet]['transition_distance'] - mapgraph.node[prevStreet]['length']) <= transDistThresh:
                        connected = True
                        break
                if not connected:
                    rmRoads.add(nextStreet)
                
#            print 'forward pruning {1} at {0}'.format(tInd+1,len(rmRoads))
            for delStreet in rmRoads:
                del roadSetSeq[tInd+1][delStreet]
                    
            if len(rmRoads) == 0:
                break
                
        # Prune backwards
        for tInd in range(minErrInd,0,-1):
            prevRoads = roadSetSeq[tInd-1]
            nextRoads = roadSetSeq[tInd]
            if len(prevRoads) == 1:
                break

            rmRoads = set()
            for prevStreet in prevRoads.iterkeys():
                connected = False
                for nextStreet in nextRoads.iterkeys():
                    if mapgraph.has_edge(prevStreet,nextStreet) and (mapgraph[prevStreet][nextStreet]['transition_distance'] - mapgraph.node[prevStreet]['length']) <= transDistThresh:
                        connected = True
                        break
                if not connected:
                    rmRoads.add(prevStreet)
                
#            print 'back pruning {1} at {0}'.format(tInd-1,len(rmRoads))
            for delStreet in rmRoads:
                del roadSetSeq[tInd-1][delStreet]
                    
            if len(rmRoads) == 0:
                break
    #print 'done.'
    
    prevStreet = None
    prevPosition = None
    prevOrient = None
    currStreet = None
    currPosition = None
    currOrient = None

    if data_skip == None or data_skip < 1:
        data_skip = 1
    else:
        mapdynamics.setParameters({'dt':dt*data_skip,'dt_1':dt*data_skip})

    debugStreet = None
    
    states = []
    headingErrSum = 0
    posErrSum = 0
    numErrs = 0
    #print 'Converting to street state...'
    for tInd in range(len(roadSetSeq)):
#        if debugStreet != currRoads.keys()[0]:
#            print '{0}: -> {1}'.format(tInd,debugStreet)
#            debugStreet = currRoads.keys()[0]
        if (tInd+1)%data_skip != 0:
            continue
#        print tInd
        currRoads = roadSetSeq[tInd]
        (gpsPos,latlon,instVF,instVel,heading) = posInfos[tInd]
        
        assert(len(currRoads) == 1)
        prev2Position = prevPosition
        prev2Orient = prevOrient
        prev2Street = prevStreet
        prevStreet = currStreet
        prevPosition = currPosition
        prevOrient = currOrient
        currStreet = currRoads.keys()[0]
        currOrient = currRoads[currStreet][2]
        currPosition = currRoads[currStreet][3]
        
        projPos = mapgraph.get_road_position(currStreet,np.array([currPosition]))
        posErrSum += np.sqrt(np.sum(np.power(projPos.reshape(2) - gpsPos.reshape(2),2.0)))
        numErrs += 1
        
        if prevStreet != None and not mapgraph.has_edge(prevStreet,currStreet):
            print 'ERROR: missing link at tInd = {0}'.format(tInd)
            print 'prevStreet:'
            mapgraph.printNode(prevStreet)
            print '\ncurrStreet:'
            mapgraph.printNode(currStreet)
            assert(prevStreet == None or mapgraph.has_edge(prevStreet,currStreet))
        

        if currStreet != prevStreet:
            if prevStreet != None:
                prevOrient = prevOrient - mapgraph[prevStreet][currStreet]['angle']
                prevPosition = prevPosition - mapgraph.node[prevStreet]['length']
                prev2Orient = prev2Orient - mapgraph[prevStreet][currStreet]['angle']
                prev2Position = prev2Position - mapgraph.node[prevStreet]['length']
            else:
                prevOrient = currOrient
                prevPosition = currPosition - mapdynamics.dt*instVF
                prev2Orient = currOrient
                prev2Position = currPosition - 2*mapdynamics.dt*instVF

        stateVec = mapdynamics.convert_posorient_sequence(mapgraph,[(prev2Street,prev2Position,prev2Orient),(prevStreet,prevPosition,prevOrient),(currStreet,currPosition,currOrient)])
        states.append({'street':currStreet,'state':stateVec[-1],'gps_position':gpsPos,'gps_heading':heading,'map_position':projPos,'latlon':latlon})

#    print 'Map projection error: {0}m'.format(1000.0*posErrSum/numErrs)

    return states

def do_convert_gps_data(options,cfgFile):
    load_config_file(options,cfgFile)
    mapgraph,mapVersion = load_mapgraph(options)
    mapdynamics = load_mapdynamics(options)

    states = convert_gps_trajectory(mapgraph,mapdynamics,options.gps_data,options.dt,options.data_skip)

    outName = get_mapsequencedyn_outfile_path(options,'gt_states.p',includeCrop = False)
    pickle.dump(states, open(outName,'wb'))

def do_synthesize_odometry(options,args):
    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    if size > 1:
        if rank == 0:
            print 'Warning: synthesize_odometry does not exploit MPI parallelization'
        else:
            return
    
    assert len(args) == 4, 'Wrong number of arguments for synthesize_odometry.'

    obsFile = args[0]
    assert obsFile.endswith('.obs')

    cfgFile = args[1]
    assert cfgFile.endswith('.dcfg')
    
    posSigma = float(args[2])
    angSigma = float(args[3])
    
    load_config_file(options,cfgFile)
    mapgraph,mapVersion = load_mapgraph(options)
    mapdynamics = load_mapdynamics(options)
    
    # These signal parameters are specific to the data we're currently using
    signalMag = (0.01067254,  0.16930674)

#    obs = np.array([float(splitLine[0])/options.dt, float(splitLine[1])/options.dt]).reshape(2,1)
#    int_pos = np.array([float(splitLine[2]), float(splitLine[3])]).reshape(2,1)
#    int_ang = splitLine[4]
    projScale = mapgraph.graph['mercator_scale']
    ptCoordOrigin = mapgraph.graph['position_offset']

    latlon = np.empty(2)

    obsFile = open(obsFile,'wt')
    gpsFile = open(options.gps_data,'rt')
    for (tInd,line) in enumerate(gpsFile):
        lineParts = line.split()
        
        latlon[0] = float(lineParts[0])
        latlon[1] = float(lineParts[1])        
        pos = mercatorProj(latlon,projScale) - ptCoordOrigin
        heading = float(lineParts[5])
        
        if tInd == 0:
            basePos = copy(pos)
            baseHeading = copy(heading)
            baseR = np.array([[np.cos(heading - np.pi/2.0), np.sin(heading - np.pi/2.0)], [-np.sin(heading - np.pi/2.0), np.cos(heading - np.pi/2.0)]])
            prevPos = copy(pos)
            prevHeading = copy(heading)
            continue
        
        dHeading = (heading - prevHeading)
        while dHeading < -np.pi: dHeading += 2.0*np.pi
        while dHeading > np.pi: dHeading += -2.0*np.pi
        dPos = (pos - prevPos)*1000.0
        cdir = np.array([np.cos(prevHeading),np.sin(prevHeading)])
        int_pos = np.dot(baseR,pos - basePos)
        
        posNoise = options.dt*1000.0*posSigma*signalMag[0]*np.random.normal()
        angNoise = options.dt*angSigma*signalMag[0]*np.random.normal()

        obsFile.write('{0} {1} {2} {3}\n'.format(np.dot(cdir,dPos) + posNoise,dHeading + angNoise,int_pos[0],int_pos[1]))

        prevPos = copy(pos)
        prevHeading = copy(heading)

    obsFile.close()
    gpsFile.close()


def load_gt_data(options):
    return pickle.load(open(get_mapsequencedyn_outfile_path(options,'gt_states.p',includeCrop = False),'rb'))

def do_fit_motionmodel(options,args):
    if options.mpi:
        comm = MPI.COMM_WORLD
        size = comm.Get_size()
        rank = comm.Get_rank()
    else:
        size = 1
        rank = 0

    if size > 1:
        if rank == 0:
            print 'Warning: fit_motionmodel does not exploit MPI parallelization'
        else:
            return
 
    assert len(args) > 1, 'Need at least one data file'

    outFileName = args[0]
    assert outFileName.endswith('.p'), 'First argument must be the output pickle file for the fit parameters'

    mapdynamics = load_mapdynamics(options)

    allStates = list()
    for cfgFile in args[1:]:
        print 'Loading data from {0}...'.format(cfgFile),

        cOptions = deepcopy(options)
        load_config_file(cOptions,cfgFile)

        mapgraph,mapVersion = load_mapgraph(cOptions)

        obs = load_observations(cOptions)
        gtData = load_gt_data(cOptions)

        allStates.append((mapgraph,obs,gtData))
 
        print 'done.'
    
    print 'Fitting parameters...',
    dynParams = mapdynamics.fit(allStates)
    print 'done.'
    
    pickle.dump(dynParams,open(outFileName,'wb'))
    print 'Saved motion model to {0}'.format(outFileName)

def do_plot_odometry(options,args):
    assert(len(args) == 1)
    load_config_file(options,args[0])

    import matplotlib.pyplot as plt

    obs = load_observations(options)
        
    D = []
    T = []
    for curr_obs in obs:
        t = curr_obs['t']
        yt = curr_obs['obs']

        T.append(t)
        D.append(yt)

    time_mat = np.array(T)
    obs_mat = np.array(D)
    
    if options.gps_data != None:
        mapgraph,mapVersion = load_mapgraph(options)
        mapdynamics = load_mapdynamics(options)

        states = load_gt_data(options)
        S = []
        TransPreds = []
        TransVals = []
        prevState = None
        for currState in states:
            (ymu,ySigma) = mapdynamics.observation_distribution(mapgraph,currState['street'],currState['state'])
            S.append(ymu)
            if prevState != None:
                svMat = mapdynamics.get_state_vel_matrix(mapgraph,currState['street'])
                (smu,sSigma) = mapdynamics.state_transition_distribution(mapgraph,currState['street'],prevState['street'],prevState['state'])
                TransPreds.append(np.dot(svMat,smu))
                TransVals.append(np.dot(svMat,currState['state']))
            prevState = currState
        gtObsMu = np.array(S)
        gtState = np.array(TransVals)
        gtStatePreds = np.array(TransPreds)

        plt.figure(1)
        plt.subplot(2,1,1)
        plt.plot(time_mat,60*60*obs_mat[:,0],time_mat,60*60*gtObsMu[:,0])
        plt.legend(['Observations','GPS'])
        plt.title('Forward Velocity (km/hr)')
        
        plt.subplot(2,1,2)
        plt.plot(time_mat,obs_mat[:,1],time_mat,gtObsMu[:,1])
        plt.legend(['Observations','GPS'])
        plt.title('Angular change (rad)')
        
        plt.figure(2)
        plt.subplot(2,1,1)
        plt.plot(time_mat[1:],gtState[:,0],time_mat[1:],gtStatePreds[:,0])
        plt.legend(['GT Data','Prediction'])
        plt.title('Linear Position')

        plt.subplot(2,1,2)
        plt.plot(time_mat[1:],gtState[:,2],time_mat[1:],gtStatePreds[:,2])
        plt.legend(['GT Data','Prediction'])
        plt.title('Orientation')
    else:
        plt.figure(1)
        plt.subplot(2,1,1)
        plt.plot(time_mat,60*60*obs_mat[:,0])
        plt.legend(['Observations'])
        plt.title('Forward Velocity (km/hr)')
        
        plt.subplot(2,1,2)
        plt.plot(time_mat,obs_mat[:,1])
        plt.legend(['Observations'])
        plt.title('Angular change (rad)')
    
    plt.show()
    
    
def do_print_gps_chance(options,args):
    errs = []
    for cfgFile in args:
        print 'Loading data from {0}...'.format(cfgFile),

        cOptions = deepcopy(options)
        load_config_file(cOptions,cfgFile)

        mapgraph,mapVersion = load_mapgraph(cOptions)
        gtData = load_gt_data(cOptions)
        
        cErrs = [] 
        for currGt in gtData:
            cErrs.append(np.sqrt(np.sum(np.power(currGt['gps_position'].reshape(2) - mapgraph.posAvg.reshape(2),2.0))))
        errs += cErrs
        cErrs = np.array(cErrs)

        print '  Chance Error: {0}m'.format(1000.0*np.mean(cErrs))
    
    print '  Average Chance Error: {0}m'.format(1000.0*np.mean(np.array(errs)))
        
    

def load_config_file(cmdOpts,cfgFile):
    dataOptions = {'gps_data':'str','data_dir':'str','odometry':'str','map':'str', \
                   'out_prefix':'str','dt':'float','syn_path':'str','syn_speed':'float', \
                   'start_frame':'int','data_skip':'int','dataname':'str', \
                   'dynamics':'str','dynamics_params':'str','crop_size':'float', \
                   'sequence_id':'str', 'data_source':'str', 'snr':'float'}
    cfgFileParser = SafeConfigParser()
    cfgFileParser.read(cfgFile)
    
    for (name,val) in cfgFileParser.items('sequence_data'):
        assert name in dataOptions, 'Unrecognized parameter option {0} in config file {1}'.format(name,cfgFile)
        if hasattr(cmdOpts,name) and getattr(cmdOpts,name) == None:
            if dataOptions[name] == 'str':
                setattr(cmdOpts,name,val)
            elif dataOptions[name] == 'int':
                setattr(cmdOpts,name,int(val))
            elif dataOptions[name] == 'float':
                setattr(cmdOpts,name,float(val))
            else:
                assert(False)
                
    if cmdOpts.dynamics == None or cmdOpts.dynamics == '':
        cmdOpts.dynamics = 'state2rigid' 

def get_mapsequencedyn_outfile_path(options,filename,includeCrop):
    mapName = get_map_name(options.map)
    if includeCrop and options.crop_size != None:
        mapName += 'crop{0}'.format(options.crop_size)

    if options.out_dir == None:
        base_out_dir = ''
    else:
        base_out_dir = options.out_dir

    if options.out_prefix == None:
        out_prefix = ''
    else:
        out_prefix = options.out_prefix
        
    seqName = options.sequence_id

    if options.dynamics == None:
        dynName = 'state2rigid'
    else:
        dynName = options.dynamics
    
    outdir = os.path.join(base_out_dir, \
                          '{0}sequence{1}-map{2}-dyn{3}'.format(out_prefix,seqName,mapName,dynName))
    mkdir_p(outdir)
    outname = os.path.join(outdir,filename)
    return outname

def get_outfile(options,suffix,includeCrop = True):
    mapName = get_map_name(options.map)
    if includeCrop and options.crop_size != None:
        mapName += 'crop{0}'.format(options.crop_size)

    if options.out_dir == None:
        base_out_dir = ''
    else:
        base_out_dir = options.out_dir

    if options.out_prefix == None:
        out_prefix = ''
    else:
        out_prefix = options.out_prefix
        
    if options.dataname == None:
        dataname = ''
    else:
        dataname = options.dataname

    if options.dynamics == None:
        dynName = 'state2rigid'
    else:
        dynName = options.dynamics

    if options.simplify_posterior:
        if options.simplify_thresh_name:
            simplifyStr = 'simplifyThresh{0}'.format(options.simplify_threshold)
        else: 
            simplifyStr = 'simplify1'
    else: 
        simplifyStr = 'simplify0'

    outname = os.path.join(base_out_dir, \
                          '{0}data{1}-'.format(out_prefix,dataname) + \
                          'map{1}-dyn{2}-filter{3}-{4}-{5}'.format(options.data_skip,mapName,dynName,options.filter_version,simplifyStr,suffix))
    return outname
    
def get_outfile_path(options,filename,includeCrop = True):
    mapName = get_map_name(options.map)
    if includeCrop and options.crop_size != None:
        mapName += 'crop{0}'.format(options.crop_size)

    if options.out_dir == None:
        base_out_dir = ''
    else:
        base_out_dir = options.out_dir

    if options.out_prefix == None:
        out_prefix = ''
    else:
        out_prefix = options.out_prefix
        
    if options.dataname == None:
        dataname = ''
    else:
        dataname = options.dataname
        
    if options.dynamics == None:
        dynName = 'state2rigid'
    else:
        dynName = options.dynamics
        
    if options.simplify_posterior:
        if options.simplify_thresh_name:
            simplifyStr = 'simplifyThresh{0}'.format(options.simplify_threshold)
        else: 
            simplifyStr = 'simplify1'
    else: 
        simplifyStr = 'simplify0'

    outdir = os.path.join(base_out_dir, \
                          '{0}data{1}'.format(out_prefix,dataname), \
                          'skip{0}-map{1}-dyn{2}-filter{3}-{4}'.format(options.data_skip,mapName,dynName,options.filter_version,simplifyStr))
    mkdir_p(outdir)
    outname = os.path.join(outdir,filename)
    return outname

def get_datafile_path(options):
    return get_outfile_path(options,'data.p')

cmdLineParser = OptionParser()
cmdLineParser.add_option('--mode',type='string',dest='mode', \
                  help='One of filter, display, plot_data, convert_map or display_map')

cmdLineParser.add_option('-m','--map',type='string',dest='map', \
                  help='map to use, can be a file name (.bin or .p) or an integer to specify toy maps')
cmdLineParser.add_option('--dataname',type='string',dest='dataname', \
                  help='name for data file to use.  If not specified, defaults to empty')
cmdLineParser.add_option('--gps_data',type='string',dest='gps_data', \
                  help='GPS ground truth data file')
cmdLineParser.add_option('--odometry',type='string',dest='odometry', \
                  help='input file of observations to use.  if unspecified, data is synthesized')
cmdLineParser.add_option('--dt',type='float',dest='dt') # Time is in seconds
cmdLineParser.add_option('--snr',type='float',dest='snr')
cmdLineParser.add_option('--syn_path',type='string',dest='syn_path')
cmdLineParser.add_option('--syn_speed',type='float',dest='syn_speed') # In km/s
cmdLineParser.add_option('--start_frame',type='int',dest='start_frame', \
                  help='Frame number on which to start processing data')
cmdLineParser.add_option('--out_prefix',type='string',dest='out_prefix')
cmdLineParser.add_option('--data_source',type='string',dest='data_source') # only used for making plots
cmdLineParser.add_option('--sequence_id',type='string',dest='sequence_id') # only used for making plots
cmdLineParser.add_option('--data_skip',type='int',dest='data_skip', \
                         help='how many frames of data to skip/integrate between observations')
cmdLineParser.add_option('--dynamics',type='string',dest='dynamics', \
                  help='Which form of state dynamics to use, currently only state2rigid is available')
cmdLineParser.add_option('--dynamics_params',type='string',dest='dynamics_params', \
                         help='parameter pickle file of dynamics parameters')
cmdLineParser.add_option('--crop_size',type='float',dest='crop_size', \
                         help='crop the initialization to a region this size (in km) around the ground truth starting position')

cmdLineParser.add_option('--out_dir',type='string',dest='out_dir',default='results')
cmdLineParser.add_option('--debug_map',type='int',dest='debug_map')
cmdLineParser.add_option('-s','--speed',type='float',dest='speed',default=40.0/3600.0) # In km/s
cmdLineParser.add_option('--img_format',type='string',dest='img_format',default='png')
cmdLineParser.add_option('--seed',type='int',dest='seed',default=0)
cmdLineParser.add_option('--simplify_map',type='int',dest='simplify_map',default=True)
cmdLineParser.add_option('--filter',type='int',dest='filter_version',default=3, \
                  help='Which version of the filtering algorithm to use, currently the best is 3')
cmdLineParser.add_option('--simplify',type='int',dest='simplify_posterior',default=1, \
                  help='Turn on simplification or not')
cmdLineParser.add_option('--simplify_thresh_name',type='int',dest='simplify_thresh_name',default=0)
cmdLineParser.add_option('--simplify_threshold',type='float',dest='simplify_threshold',default=0.01, \
                  help='The maximum error to accept in simplification (in nits)')
cmdLineParser.add_option('--mpi',type='int',dest='mpi', \
                  help='Enable MPI based parallelization')
cmdLineParser.add_option('--load_balance_freq',type='int',dest='load_balance_freq',default=1, \
                  help='Set load MPI rebalancing frequency (1 default,0 disables rebalancing)')
cmdLineParser.add_option('--resume',type='int',dest='resume',default=0, \
                  help='Frame number on which to resume processing data')
cmdLineParser.add_option('--enable_postprint',action='store_false',dest='disable_postprint',default=True, \
                  help='Enable printing of the posterior')
(options, args) = cmdLineParser.parse_args()

if options.mpi == None:
    options.mpi = 'PMI_RANK' in os.environ and 'PMI_SIZE' in os.environ

if options.mpi:
    from mpi4py import MPI

    comm = MPI.COMM_WORLD
    size = comm.Get_size()
    rank = comm.Get_rank()
else:
    size = 1
    rank = 0


#np.seterr(divide='raise')
#np.random.seed(options.seed)
#random.seed(options.seed)

cmds = { 'filter':do_inference, 'plot_snr_errors':do_plot_snr_errors, 'plot_crop_errors':do_plot_crop_errors, \
         'latex_tables':do_latex_tables, 'plot_errors':do_plot_errors, 'synthesize_odometry':do_synthesize_odometry, \
         'plot_odometry':do_plot_odometry, 'fit_motionmodel':do_fit_motionmodel, 'plot_natthresh_errors':do_plot_natthresh_errors, \
         'print_map_stats':do_print_map_stats, 'print_gps_chance':do_print_gps_chance, 'print_timing_stats':do_print_timing_stats, \
         'display_frame':do_display_frame }

if options.mode in cmds:
    cmds[options.mode](options,args)

elif options.mode == 'plot_data':
    do_plot_data(options)

elif options.mode == 'display_gt':
    do_display_gt(options)

elif options.mode == 'make_movie':
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        do_make_movie(copy(options),args[i])
        print '{0} done making movie.'.format(args[i])

elif options.mode == 'display':
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    for cfgFile in args:
        do_display(copy(options),cfgFile,False)
        if rank == 0:
            print '{0} done display posteriors.'.format(cfgFile)

elif options.mode == 'display_crop':
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    for cfgFile in args:
        do_display(copy(options),cfgFile,True,False)
        if rank == 0:
            print '{0} done display cropped posteriors.'.format(cfgFile)

elif options.mode == 'display_croptrack':
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    for cfgFile in args:
        do_display(copy(options),cfgFile,True,True)
        if rank == 0:
            print '{0} done display cropped, tracked posteriors.'.format(cfgFile)

elif options.mode == 'plot_stats':
    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        do_plot_stats(copy(options),args[i])
        print '{0} done plotting stats.'.format(args[i])

elif options.mode == 'extract_modes':
    modeInfo = {}
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        print 'Extracting modes from {0}...'.format(args[i])
        do_extract_modes(copy(options),args[i])
        print '{0} done extracting modes.'.format(args[i])

elif options.mode == 'compute_errors':
    if args[0].endswith('.dcfg'):
        srcName = 'default'
    else:
        srcName = args[0]
        args = args[1:]

    modeInfo = {}
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        modeInfo[args[i]] = do_compute_errors(copy(options),args[i],topErrNs)
        print '{0} done computing errors.'.format(args[i])
elif options.mode == 'convert_gps_data':
    modeInfo = {}
    for i in range(rank,len(args)+size,size):
        if i >= len(args):
            break
        do_convert_gps_data(deepcopy(options),args[i])
        print 'done converting data for {0}.'.format(args[i])
elif options.mode == 'print_odometry':
    allObs = []
    for odomFile in args:
        allObs.append(load_observations_file(odomFile,options.dt,options.data_skip))
        
    for (tInd,cAllObs) in enumerate(izip(*allObs)):
        print 'tInd = {0}'.format(cAllObs[0]['tInd'])
        fields = ['obs', 'odom_int_pos', 'odom_int_ang']
        for f in fields:
            print '{0}:'.format(f)
            for obs in cAllObs:
                print obs[f]

elif options.mode == 'print_odometry_stats':
    obsSqSum = np.zeros(2)
    obsN = 0
    for odomFile in args:
        cSeq = load_observations_file(odomFile,options.dt,options.data_skip)

        cobsSqSum = np.zeros(2)
        cobsN = 0

        for cobs in cSeq:
            cobsSqSum += np.power(cobs['obs'].reshape(2),2.0)
            cobsN += 1

        print '{0} signal: {1}'.format(odomFile,np.sqrt(cobsSqSum/cobsN))
        obsSqSum += cobsSqSum
        obsN += cobsN
    print 'Avg signal: {0} (N = {1})'.format(np.sqrt(obsSqSum/obsN),obsN)
        
    
elif options.mode == 'convert_map':
    if options.debug_map == None:
       options.debug_map = True 

    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    mapgraph,mapVersion = load_mapgraph(options)
    mapName = get_map_name(options.map)
    mapSz = mapgraph.posMax - mapgraph.posMin
    if options.simplify_map:
        mapOutName = '{0}.p'.format(mapName)
        mapDisplayName = '{0}.{1}'.format(mapName,options.img_format)
    else:
        mapOutName = '{0}-nosimplify.p'.format(mapName)
        mapDisplayName = '{0}-nosimplify.{1}'.format(mapName,options.img_format)
    pickle.dump(mapgraph,open(mapOutName,'wb'))
    print 'Wrote converted map to {0}'.format(mapOutName)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim((mapgraph.posMin[0] - 0.1*mapSz[0],mapgraph.posMax[0] + 0.1*mapSz[0]))
    ax.set_ylim((mapgraph.posMin[1] - 0.1*mapSz[1],mapgraph.posMax[1] + 0.1*mapSz[1]))
    mapgraph.display(fig, ax, debugDisplay = options.debug_map)
    print 'Displaying map to {0}...'.format(mapDisplayName)
    fig.savefig(mapDisplayName)
    print 'done.'
elif options.mode == 'display_map':
    if options.debug_map == None:
       options.debug_map = True 

    import matplotlib
    if options.img_format == 'pdf':
        matplotlib.use('pdf')
    else:
        matplotlib.use('agg')
    import matplotlib.pyplot as plt

    mapgraph,mapVersion = load_mapgraph(options)
    mapName = get_map_name(options.map)
    mapOutName = '{0}.{1}'.format(mapName,options.img_format)
    mapSz = mapgraph.posMax - mapgraph.posMin

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim((mapgraph.posMin[0] - 0.1*mapSz[0],mapgraph.posMax[0] + 0.1*mapSz[0]))
    ax.set_ylim((mapgraph.posMin[1] - 0.1*mapSz[1],mapgraph.posMax[1] + 0.1*mapSz[1]))
    mapgraph.display(fig, ax, debugDisplay = options.debug_map)
    print 'Saving map to {0}...'.format(mapOutName)
    fig.savefig(mapOutName)
    print 'done.'
elif options.mode == 'profile_filter':
    import cProfile
    cProfile.run('do_inference(options,args)','profile_filter.stats')
elif options.mode == 'print_mapnode':
    mapgraph,mapVersion = load_mapgraph(options)

    for nodeStr in args:
        nodes = mapgraph.matchNode(nodeStr)
        for i in nodes:
            mapgraph.printNode(i)

elif options.mode == 'print_mapedge':
    mapgraph,mapVersion = load_mapgraph(options)

    for edgeStr in args:
        nodeStrs = edgeStr.split(',')
        startNodes = mapgraph.matchNode(nodeStrs[0])
        endNodes = mapgraph.matchNode(nodeStrs[1])

        for sn in startNodes:
            for en in endNodes:
                if mapgraph.has_edge(sn,en):
                    mapgraph.printEdge(sn,en)
else:
    assert False, 'Unknown command mode: {0}'.format(options.mode)

