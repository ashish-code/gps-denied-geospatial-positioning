'''
Created on Jun 25, 2015

@author: ash
'''

import networkx as nx
import matplotlib.pyplot as plt
# networkx adjacency is stored as a dictionary of dictionary

# networkx basic graph data structure is a dictionaries of dictionaries of dictionary


def simpleCreateGraph():
    myGraph1 = nx.Graph()
    myGraph1.add_edge(1,2)
    myGraph1.add_edge(2,3,weight=0.9)
    
    nx.draw_networkx(myGraph1, pos=None, with_labels=True)
    plt.show()
    pass

def weightedGraph():
    myG = nx.Graph()
    edgeList = [('a','b',0.4), ('b','c',0.7), ('c','a',0.1), ('c','d',0.8)] # list of tuples
    myG.add_weighted_edges_from(edgeList)
    
    nx.draw_networkx(myG, pos=None, with_labels=True)
    plt.show()
    pass

def adjGraph():
    adjG = nx.Graph()
    adjG.add_edge('A','B')
    adjG.add_edge('B','C')
    adjG.add_edge('C', 'A')
    
    nx.draw_networkx(adjG, pos=None, with_labels=True)
    plt.show()
    print adjG.adj
    pass

def graphType():
    
    pass

if __name__ == '__main__':
#     simpleCreateGraph()
#     weightedGraph()
    adjGraph()
    pass