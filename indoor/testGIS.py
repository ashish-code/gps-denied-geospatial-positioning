'''
Created on Sep 29, 2015

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

shpRoomURL = "/home/ash/Dropbox/OSU/BIM/osuSmithLabShpFile/065-0b.shp"
shpTrajectoryURL = "/home/ash/Dropbox/OSU/BIM/osuSmithLabShpFile/osusmith065-0b.shp"

shpFileURL = "/home/ash/Data/FixedShapeFiles/osusmithlabfloor02/065-02lines.shp"


def showRoomNTrajectories():
    roomd = nx.read_shp(shpRoomURL)
    trajectoryd = nx.read_shp(shpTrajectoryURL)
    
    room = roomd.to_undirected()
    trajectory = trajectoryd.to_undirected()
    
    print room.nodes(data=False)
    print trajectory.nodes(data=False)
    
    nodeList = room.nodes(data=True)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode): pos.append(nodeList[i][0])
    roomLayout = dict(zip(room,pos))
    
    nodeList = trajectory.nodes(data=True)
    nNode = len(nodeList)
    pos = []
    for i in xrange(nNode): pos.append(nodeList[i][0])
    trajectoryLayout = dict(zip(trajectory, pos))
    
    plt.figure(1)
    nx.draw_networkx_edges(room, pos=roomLayout, edgelist=None, width=2, edge_color='k', style='-', alpha=0.8)
    nx.draw_networkx_edges(trajectory, pos = trajectoryLayout, edgelist=None, width=1, edge_color='b', style='-', alpha=0.2)
    plt.show()
    pass

def showShpFileData(shpFileURL):
    shpData = nx.read_shp(shpFileURL)
    shpData = shpData.to_undirected()
    
    print shpData.nodes(data=True)
    for edge in shpData.edges(data=True):
        print edge
    
    nodeList = shpData.nodes(data=True)
    nNode = len(nodeList)
    
    pos = []
    for i in xrange(nNode):
        pos.append(nodeList[i][0])
        pass
    
    shpLayout = dict(zip(shpData,pos))
    
    plt.figure(1)
    nx.draw_networkx_edges(shpData, pos=shpLayout, edgelist=None, width=1, edge_color='b', style='-', alpha=0.5)
    nx.draw_networkx_nodes(shpData, pos=shpLayout, nodelist=None, node_size=2)
    
    print len(shpData.nodes(data=False))
    print len(shpData.edges(data=False))
    
    plt.show()


if __name__ == '__main__':
#     showRoomNTrajectories()
    showShpFileData(shpFileURL)
    pass