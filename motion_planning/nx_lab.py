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
#matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree
import networkx as nx
from IPython import get_ipython
import time

#from enum import Enum
from queue import PriorityQueue

import math
from collections import Counter

""" get_ipython().run_line_magic('matplotlib', 'inline')
plt.switch_backend('Qt5agg')

plt.rcParams['figure.figsize'] = 12, 12
 """
print ("pos")

class RRT:
    
    
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
        """
        Returns a grid representation of a 2D configuration space
        based on given obstacle data, drone altitude and safety distance
        arguments.
        """


    def __init__(self, x_init, weight, G):
            # A tree is a special case of a graph with
            # directed edges and only one path to any vertex.
            self.tree = nx.DiGraph()
            self.tree.add_node(x_init)
            self.graph()
    
    def graph(self, weight, G):
        self.tree.add_edge("A", "B", weight-4)
        self.tree.add_edge("B", "D", weight-4)
        self.tree.add_edge("A", "C", weight-4)
        self.tree.add_edge("C", "D", weight-4)

        nx.shortest_path(G, "A", "D", weight="weight")
        ['A', 'B', 'D']

        pos = nx.circular_layout(self.tree)
        weights = [wt for u, v, wt in G.edges(data="weight")]
        nx.draw_networkx(self.tree, pos, width=weights)

        labels = nx.get_node_attributes(self.tree, "weight")
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

        print ("pos")
            
        