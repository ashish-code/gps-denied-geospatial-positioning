'''
Created on Jun 26, 2015

@author: ash
'''

import shapefile as sf
dataURL = "/home/ash/Data/tl_2014_39049_roads/"
dataName = "tl_2014_39049_roads"
dbSuffix = ".dbf"
shpSuffix = ".shp"

dbfFileURL = dataURL + dataName + dbSuffix
shpFileURL = dataURL + dataName + shpSuffix

def readShpDB():
    reader = sf.Reader()
    pass

if __name__ == '__main__':
    readShpDB()
    pass