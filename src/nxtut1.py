'''
Created on Jun 24, 2015

@author: ash
'''

import networkx as nx
import matplotlib.pyplot as plt

sn = nx.Graph()
sn.add_node(1, nlabel="a")
sn.add_node(2, nlabel="b")
sn.add_edge(1, 2, elabel="a-b")
sn.add_node(3, nlabel="c")
nx.draw(sn)
plt.show()
