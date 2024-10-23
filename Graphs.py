import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib
from Colors import *

matplotlib.use("Agg")
import matplotlib.backends.backend_agg as agg


def initialize(matrix):
    '''Parameters
    -----------------------
         matrix: a numpy array stored adjacency matrix.
    -----------------------
    Return: 
        G: networkX graph.
        pos: vertice positions.
        color_map: map color of each node.
    '''
    n_vertices=matrix.shape[0]
    
    G=nx.DiGraph()
    for row in range(n_vertices):
        for col in range(n_vertices):
            w=matrix[row][col]
            if w!=0: G.add_edge(row, col, color='black', weight=matrix[row][col])

    pos = nx.spring_layout(G)  # positions for all nodes
    
    color_map={node: default_color for node in G.nodes}#change color 

    return G, pos, color_map
    
    

def generateFigure(G, pos, color_map):
    
    fig=plt.figure(figsize=(8, 8), dpi=100)
  
    nx.draw_networkx_nodes(G, pos, node_color=list(color_map.values()),node_size=800)

    # edges
    edges = G.edges()

    colors = [G[u][v]['color'] for u,v in edges]
    nx.draw_networkx_edges(G, pos, edgelist=G.edges, edge_color =colors, arrowstyle="->",arrowsize=20, width=2)

    # labels
    labels = nx.get_edge_attributes(G,'weight')
    nx.draw_networkx_labels(G,pos, font_size=20, font_family="sans-serif")
    nx.draw_networkx_edge_labels(G,pos, edge_labels=labels, font_size=12, font_family="sans-serif")
    plt.axis("off")
    
    return fig

def getRawData(fig):
    canvas = agg.FigureCanvasAgg(fig)
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()
    size = canvas.get_width_height()
    return raw_data, size

def readMatrix(input):
    with open(input,'rt') as f:
        l=0
        matrix=[]
        for line in f:
            if l==0:
                tmp=line.split()
                start = int(tmp[0])
                end = int(tmp[1])
            else:
                matrix.append(line.split())
            l+=1
        
    matrix=np.array(matrix).astype(int)

    return matrix, start, end
