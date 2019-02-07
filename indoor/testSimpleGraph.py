'''
Created on Jun 24, 2016

@author: ash
'''

#The package which handles the graph objects
import networkx as nx

# Matplotlib is the default package for
# rendering the graphs
import matplotlib.pyplot as plt
import matplotlib.colors as clrs
def simple_graph():

    #create an empty graph
    G = nx.Graph()
    
    #add three edges
    G.add_edge('A','B');
    G.add_edge('B','C');
    G.add_edge('C','A');
    
    cmap = plt.cm.get_cmap('jet')
    
    print clrs.colorConverter.to_rgb(cmap(0.5))
    

    #draw the graph
#     nx.draw(G, width=4, edge_color=(cmap(0.5), cmap(0.25), cmap(0.75)))
    nx.draw(G, width=4, node_color='g', edge_color = clrs.rgb2hex(clrs.colorConverter.to_rgb(cmap(0.5))))
    
    

    #show
    plt.show()

simple_graph()