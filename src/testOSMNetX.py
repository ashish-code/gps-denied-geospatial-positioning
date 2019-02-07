'''
Created on Jun 29, 2015

@author: ash
'''

import os
import networkx as nx
from pyNetXOsm import *
dataURL = "/home/ash/Data/"
dataFile = "osu.osm"

dataPath = os.path.join((dataURL, dataFile))

def osmio():
    osuG = read_osm(dataPath, True)
    print osuG
    pass

if __name__ == '__main__':
    osmio()
    pass