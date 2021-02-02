#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 25 13:27:30 2021

@author: adarsh
"""


import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue

G = nx.Graph()
opened = PriorityQueue()
closed = []

nodes = [('S', 8), ('A', 10), ('B', 5), ('C', 7), ('D', 3), ('E', 3), ('F', 1), ('G', 0), ('H', 5), ('I', 6)]


for node in nodes:
    G.add_node(node[0], heuristic=node[1])
    
edges = [('S', 'A', 6), ('S', 'B', 3), ('A', 'I', 5), ('B', 'C', 1), ('B', 'D', 5), ('C', 'D', 8), ('D', 'F', 5), ('D', 'E', 5), ('E', 'F', 3), ('F', 'G', 2), ('F', 'H', 3), ('H', 'I', 1), ('G', 'I', 7)]

for edge in edges:
    G.add_edge(edge[0], edge[1], weight=edge[2])

nx.draw(G, with_labels=1, font_weight='bold', node_size=800)


plt.show()

def get_g():
    g = 0
    if len(closed) > 1:
        for i in range(0, len(closed) - 1):
            for j in range(1, len(closed)):
                g = g + G.edges[closed[i], closed[j]]['weight']
        return g
    return 0
    


def astar_search(graph, start_node, goal_node):
    
    opened.put((G.nodes.data()[start_node]['heuristic'], start_node))
    
    while opened.qsize() > 0:
        current_node = opened.get()
        
        closed.append(current_node[1])
        
        if current_node[1] == goal_node:
            path = []
            
        neighbors = list(G.neighbors(current_node[1]))
        
        min_f = 0
        
        for neighbor in neighbors:
            g = 0
            h = 0
            f = 0
            if neighbor in closed:
                continue
            
            g = G.edges[current_node, neighbor]['weight'] + get_g()
            h = G.nodes[neighbor]['heuristic']
            f = g + h
            
            if f < min_f:
                min_f = f
                min_neighbor = neighbor
                
        opened.put((G.nodes.data()[min_neighbor]['heuristic'], min_neighbor))
            
            
            
            
            
            
            
            
            
            
            
            
            