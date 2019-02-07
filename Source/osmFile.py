'''
Created on Nov 16, 2015

@author: ash
'''


import networkx as nx
import matplotlib.pyplot as plt
import random
import pysal
from osm2nx import download_osm
from osm2nx import read_osm

osmFileURL = '/home/ash/Downloads/karlsruhe_kitti.osm'
highway_cat = 'motorway|trunk|primary|secondary|tertiary|road|residential|service|motorway_link|trunk_link|primary_link|secondary_link|teriary_link|path|footway|cycleway|motorway_junction|unclassified'
graphPicklePath = '/home/ash/Data/InputShapeFiles/karlsruheOSM/karlsruheOSM.gpickle'

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
parser.add_option('-i', '--input', type='string', metavar='input', dest='input', default='karlsruheOSM', help='input osm file data directory name')
parser.add_option('-t','--fileType', type='string', metavar='fileType', dest='fileType', default='gpickle', help='input osm data file extension type')

inputFileDir = '/home/ash/Data/InputShapeFiles/'
outputFileDir = '/home/ash/Data/FixedShapeFiles/'


def dlOSMFile():
    (options,args) = parser.parse_args(sys.argv[1:]) #@UnusedVariable
    
    inputDirURL = inputFileDir + options.input + '/'
    fileType = options.fileType
    
    if not os.path.exists(inputDirURL):
        os.mkdir(inputDirURL)
    
    
    inputFileURL = inputDirURL + options.input  + '.' + fileType
    
    osmFileURL = download_osm(8.325, 48.925, 8.50, 49.075, highway_cat)
    
    roadGraph = read_osm(osmFileURL, only_roads=True)
    roadGraph = roadGraph.to_undirected()
    
    nx.write_gpickle(roadGraph, inputFileURL)
    
    pass


if __name__ == '__main__':
    
    dlOSMFile()
    
    pass