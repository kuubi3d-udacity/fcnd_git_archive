import sys

import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import decimal


# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from operator import itemgetter
from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


# coding: utf-8

# # Rapidly-Exploring Random Tree (RRT)
# 
# Your task is to generate an RRT based on the following pseudocode:
# 
# ```
# def generate_RRT(x_init, num_vertices, dt):
#     rrt = RRT(x_init)
#     for k in range(num_vertices):
#         x_rand = sample_state()
#         x_near = nearest_neighbor(x_rand, rrt)
#         u = select_input(x_rand, x_near)
#         x_new = new_state(x_near, u, dt)
#         # directed edge
#         rrt.add_edge(x_near, x_new, u)
#     return rrt
# ```
#     
# The `RRT` class has already been implemented. Your task is to complete the implementation of the following functions:
# 
# * `sample_state`
# * `nearest_neighbor`
# * `select_input`
# * `new_state`
# 


import matplotlib
# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import networkx as nx
from IPython import get_ipython
import time

#from enum import Enum
from queue import PriorityQueue

import math
from collections import Counter

get_ipython().run_line_magic('matplotlib', 'inline')
plt.switch_backend('Qt5agg')

plt.rcParams['figure.figsize'] = 12, 12



class RRT:
    """
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

    def add_vertex(self, x_new):
        self.tree.add_node(tuple(RRT.x_init))

    def add_edge(self, x_near, x_new, u):
        self.tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)

    @property
    def vertices(self):
        return self.tree.nodes()

    @property
    def edges(self):
        return self.tree.edges()

    def create_grid(self, data, drone_altitude, safety_distance):
        
        //Returns a grid representation of a 2D configuration space
        //based on given obstacle data, drone altitude and safety distance
        //arguments.
           

    def __init__(self, x_init, weight, G):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

        self.tree.add_edge("A", "B", weight - 4)
        self.tree.add_edge("B", "D", weight - 4)
        self.tree.add_edge("A", "C", weight - 4)
        self.tree.add_edge("C", "D", weight - 4)

        nx.shortest_path(G, "A", "D", weight="weight")
        ['A', 'B', 'D']

        pos = nx.circular_layout(self.tree)
        weights = [wt for u, v, wt in G.edges(data="weight")]
        nx.draw_networkx(self.tree, pos, width=weights)

        labels = nx.get_node_attributes(self.tree, "weight")
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    """







# matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import networkx as nx
from IPython import get_ipython
import time

#from enum import Enum
from queue import PriorityQueue

import math
from collections import Counter

get_ipython().run_line_magic('matplotlib', 'inline')
plt.switch_backend('Qt5agg')

plt.rcParams['figure.figsize'] = 12, 12




 
import networkx as nx


G = nx.DiGraph()

pos = nx.circular_layout(G)
#weights = [wt for u, v, wt in G.edges(data="weight")]

nx.to_dict_of_dicts(G)
{
    'A': {'B': {'weight': 4}, 'C': {'weight': 3}},
    'B': {'B': {'weight': 4}, 'D': {'weight': 2}},
    'D': {'B': {'weight': 2}, 'C': {'weight': 4}},
    'C': {'B': {'weight': 3}, 'D': {'weight': 4}},
}

G.add_edge("A", "B", weight=4)
G.add_edge("B", "D", weight=2)
G.add_edge("A", "C", weight=3)
G.add_edge("C", "D", weight=4)

nx.shortest_path(G, "A", "D", weight="weights")
#['A', 'B', 'D']


#plt.imshow(G, cmap='Greys', origin='lower')

pos = nx.circular_layout(G)
weights = [wt for u, v, wt in G.edges(data="weight")]
nx.draw_networkx(G, pos, width=weights)#
#plt.plot(G, pos, width=weights)


labels = nx.get_node_attributes(G, "weight")

nx.draw(G)
plt.show(block=True)

#nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

"""
#plt.plot(G, pos, edge_labels=labels)
#plt.show(block=True)

# Now let's plot the generated RRT.
#sys.exit('generating waypoints')

plt.imshow(grid, cmap='Greys', origin='lower')
plt.plot(RRT.x_init[1], RRT.x_init[0], 'ro')
plt.plot(RRT.x_goal[1], RRT.x_goal[0], 'ro')

print ("rrt goal", RRT.rrt_goal)   
#plt.plot(RRT.rrt_goal[1], RRT.rrt_goal[0], 'ro')

for (v1, v2) in rrt.edges:
    plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')

plt.show(block=True)





import networkx as nx
G = nx.Graph()
G.add_edge(1, 2) # default edge data=1
G.add_edge(2, 3, weight=0.9) # specify edge data

import math
G.add_edge('y', 'x', function=math.cos)
G.add_node(math.cos) # any hashable can be a node


elist = [(1, 2), (2, 3), (1, 4), (4, 2)]
G.add_edges_from(elist)
elist = [('a', 'b', 5.0), ('b', 'c', 3.0), ('a', 'c', 1.0), ('c', 'd', 7.3)]
G.add_weighted_edges_from(elist) 




G = nx.Graph()
e = [('a', 'b', 0.3), ('b', 'c', 0.9), ('a', 'c', 0.5), ('c', 'd', 1.2)]
G.add_weighted_edges_from(e)
print(nx.dijkstra_path(G, 'a', 'd'))
['a', 'c', 'd']

plt.figure(figsize=(8, 6))
nx.draw(G)
#nx.draw(g, pos=node_positions, edge_color=edge_colors, node_size=10, node_color='black')
plt.title('Graph Representation of Sleeping Giant Trail Map', size=15)
plt.show()

"""