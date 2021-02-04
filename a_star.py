#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: 
"""


import networkx as nx
import matplotlib.pyplot as plt
from queue import PriorityQueue

G = nx.Graph()
opened = PriorityQueue()
closed = []
parent_tracker = []
path = []

nodes = [('S', 8), ('A', 10), ('B', 5), ('C', 7), ('D', 3), ('E', 3), ('F', 1), ('G', 0), ('H', 5), ('I', 6)]


for node in nodes:
    G.add_node(node[0], heuristic=node[1])
    
edges = [('S', 'A', 6), ('S', 'B', 3), ('A', 'I', 5), ('B', 'C', 1), ('B', 'D', 5), ('C', 'D', 8), ('D', 'F', 5), ('D', 'E', 5), ('E', 'F', 3), ('F', 'G', 2), ('F', 'H', 3), ('H', 'I', 1), ('G', 'I', 7)]

for edge in edges:
    G.add_edge(edge[0], edge[1], weight=edge[2])

nx.draw(G, with_labels=1, font_weight='bold', node_size=800)


plt.show()


def get_parent(current):
    for nodes in parent_tracker:
        if nodes[0] == current:
            return nodes[1]
    

def get_g(current, start_node):
    g = 0
    parent = get_parent(current)

    while parent != start_node:
        g = g + G.edges[get_parent(parent), parent]['weight']
        parent = get_parent(parent)
    return g


def astar_search(graph, start_node, goal_node):
    
    opened.put((G.nodes.data()[start_node]['heuristic'], start_node))
    
    while opened.qsize() > 0:
        
        current_node = opened.get()
        
        closed.append(current_node[1])
        
        if current_node[1] == goal_node:
            parent = get_parent(goal_node)
            if parent == None:
                return [goal_node]
            path.append(parent)
            while parent != start_node:
                path.append(get_parent(parent))
                parent = get_parent(parent)
            return list(reversed(path)) + [goal_node]
            
        neighbors = list(G.neighbors(current_node[1]))
        
        for neighbor in neighbors:
            
            
            parent_tracker.append((neighbor, current_node[1]))
            
            g = 0
            h = 0
            f = 0
            if neighbor in closed or neighbor == current_node[1]:
                continue
            
            if opened.qsize() > 0 and neighbor in opened.queue[0][1]:
                continue
            
            g = G.edges[current_node[1], neighbor]['weight'] + get_g(neighbor, start_node)
            h = G.nodes[neighbor]['heuristic']
            f = g + h
            
            opened.put((f, neighbor))
            
    return None