#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

Artificial and Computational Intelligence Assignment 8


Problem solving using A* Search Algorithm

Assignment Group 108

"""

#importing the libraries
#networkx to construct the graph 
import networkx as nx
#matplotlib to plot the graph
import matplotlib.pyplot as plt
#priority queue from queue to store the open nodes based on thier 'f' value
from queue import PriorityQueue

#creating an object of graph
G = nx.Graph()
#creating opened priority queue 
opened = PriorityQueue()
#a list to store all the closed nodes
closed = []
#a list to store immediate parents of the traversed nodes to backtrack
parent_tracker = []
#a list to store the path that led to the goal node
path = []

#list of nodes with thier heuristic values
nodes = [('S', 8), ('A', 10), ('B', 5), ('C', 7), ('D', 3), ('E', 3), ('F', 1), ('G', 0), ('H', 5), ('I', 6)]

#adding the nodes to the graph
for node in nodes:
    G.add_node(node[0], heuristic=node[1])
    
#list of edges with their weights/distance between connecting nodes    
edges = [('S', 'A', 6), ('S', 'B', 3), ('A', 'I', 5), ('B', 'C', 1), ('B', 'D', 5), ('C', 'D', 8), ('D', 'F', 5), ('D', 'E', 5), ('E', 'F', 3), ('F', 'G', 2), ('F', 'H', 3), ('H', 'I', 1), ('G', 'I', 7)]

#adding the edges to the graph
for edge in edges:
    G.add_edge(edge[0], edge[1], weight=edge[2])

#drawing the graph
nx.draw(G, with_labels=1, font_weight='bold', node_size=800)

#showing it on terminal
plt.show()

#function to iterate over the parent_tracker list and get the parent from where the node has been reached
#it accepts a node and returns the parent node
def get_parent(current):
    #iterating over the parent_tracker list
    for nodes in parent_tracker:
        #checking if the passed node matches with any node in the parent_tracker list
        if nodes[0] == current:
            #returning the parent
            return nodes[1]
    
#function to compute 'g' i.e., the cost that it took to come to the current node
#it accepts the current node and the start node
def get_g(current, start_node):
    #declaring 'g' and initializing it to 0 
    g = 0
    #getting the immediate parent of the current node and setting it to a temp variable 
    parent = get_parent(current)
    #loop to check if we have reached start node from current node and adding up the 'g' values as we traverse
    while parent != start_node:
        #adding the 'g' values by calling the 'get_parent' function to get parents 
        g = g + G.edges[get_parent(parent), parent]['weight']
        #setting temp variable to next parent
        parent = get_parent(parent)
    #returning the computed 'g' value
    return g

#function to perform A* Search Algorithm
#it accepts the graph, start node and goal node
def astar_search(graph, start_node, goal_node):
    #putting the start node into the opened priority queue
    opened.put((G.nodes.data()[start_node]['heuristic'], start_node))
    #loop until goal node is reached or all the nodes in the opened priority queue are traversed
    while opened.qsize() > 0:
        #getting the node with lowest 'f' score from the opened priority queue
        current_node = opened.get()
        #adding this node to closed list as this node will be explored now
        closed.append(current_node[1])
        #check if current node is goal node, if yes we have reached the goal node and return the path traversed
        if current_node[1] == goal_node:
            #get immediate parent of goal node
            parent = get_parent(goal_node)
            #if parent is None that means both start node and goal node is the same
            if parent == None:
                #return the goal node
                return [goal_node]
            #append parent to the path list
            path.append(parent)
            #looping over all the parents until we reach the start node
            while parent != start_node:
                #append the parents to the path list and calling 'get_parent' function to get the next parent
                path.append(get_parent(parent))
                #setting parent to the next parent
                parent = get_parent(parent)
            #returning the final path traverserd
            return list(reversed(path)) + [goal_node], current_node[0]
        #if current_node is not goal node we will explore it and get it's neighbors
        #storing the list of neighbors in a local variable
        neighbors = list(G.neighbors(current_node[1]))
        #iterating over the neighbors
        for neighbor in neighbors:
            #appending the current node to parent_tracker list as we want to track the parent with least 'f' score that led us to this neighbor
            parent_tracker.append((neighbor, current_node[1]))
            #setting temp variables to calculate 'f' value
            g = 0
            h = 0
            f = 0
            #if neighbor is already in closed list we don't have to explore it, we will exit the loop and continue with next neighbor
            if neighbor in closed or neighbor == current_node[1]:
                continue
            g = G.edges[current_node[1], neighbor]['weight'] + get_g(neighbor, start_node)
            h = G.nodes[neighbor]['heuristic']
            f = g + h
            #if neighbor is in opened list that means we have already calculated 'f' value from different path check if this new 'f' value is less
            #but first check if the opened in not empty
            if opened.qsize() > 0:
                #iterate through opened queue
                for node in opened.queue:
                    #check if neighbor is present in opened
                    if neighbor == node[1]:
                        #check if current 'f' value is less than the existing 'f' value
                        if f < node[0]:
                            #if yes push this neighbor with less 'f' value into the opened queue
                            opened.put((f, neighbor))  
                        #else exit and go to the next neighbor
                        continue
            #if it is on either in opened or closed append the neighbor along with it's 'f' value into opened
            opened.put((f, neighbor))
    #if there is no path then return None    
    return None

#main funtion
def main():
    #calling the astar_search function with 'S' as start node and 'G' as goal node
    path, cost = astar_search(G, 'S', 'G')
    print('\n')
    print('The path to goal node that Arun can take using A* Algorithm is: {} and cost is: {}'.format(path, cost))

#telling python to run the main method
if __name__ == "__main__": main()