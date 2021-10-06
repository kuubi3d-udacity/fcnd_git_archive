import queue
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

from sortedcontainers import SortedDict
from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


import matplotlib
#matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sklearn.neighbors import KDTree

import networkx as nx
import graphviz




from IPython import get_ipython
import time

#from enum import Enum
from queue import PriorityQueue

import math
from collections import Counter

get_ipython().run_line_magic('matplotlib', 'inline')
plt.switch_backend('Qt5agg')

plt.rcParams['figure.figsize'] = 12, 12



class RRT_path ():

    x_goal = (30, 750)
    rrt_goal = ()
    num_vertices = 1600
    dt = 18
    x_init = (20, 150)
    path = [(20, 30), (40, 50)]
    found = False


    # Assume all actions cost the same.

    queue = PriorityQueue()
    queue.put((0, x_goal))
    visited = set(x_goal)
    rrt_path = []
    branch = {}
     
    path_cost = 0
    g = graphviz.Digraph('RRT Path', format = 'svg', filename='hello.gv')

    def __init__(self, x_init):
        # A tree is a special case of a graph with
        ## directed edges and only one path to any vertex.
        #self.tree = nx.DiGraph()
        #self.tree.add_node(x_init)
        
        self.rrt_path = nx.DiGraph()
        self.rrt_path.add_node(x_init)

    def parent(self,current_node):
        return RRT_path.rrt_path.predecessors(current_node)
        
        
    
    
        
                
    def add_vertex(self, x_new):
        self.rrt_path.add_node(tuple(RRT_path.x_init))
    
    def add_edge(self, x_near, x_new, u):
        self.rrt_path.add_edge(tuple(x_near), tuple(x_new), orientation=u)


        
    """
    def parent(self, x_new):
        return self.tree.predecessors(x_new)

     
    def add_rrt_vertex(self, x_new):
        self.rrt_path.add_node(tuple(RRT.x_init))
    
    def add_rrt_edge(self, x_near, x_new, u):
        self.rrt_path.add_edge(tuple(x_near), tuple(x_new), orientation=u)
    """
    
    def gview(self):
        return self.gview()   

    
    @property
    def vertices(self):
        return self.tree.nodes()
    
    @property
    def edges(self):
        return self.tree.edges()

    #@property
    

        

   
    def nearest_neighbor(self, x_rand, rrt):
        
               
        #wp_radius = np.linalg.norm(x_goal)
        #print ('waypoint radius', wp_radius)
    
        closest_dist = 100000
        closest_vertex = None
        x_rand = np.array(x_rand)
       
        

        for v in rrt.vertices:
            d = np.linalg.norm(x_rand - np.array(v[:2]))
            if d < closest_dist:
                closest_dist = d
                closest_vertex = v
            '''
            if np.linalg.norm(x_goal - np.array(v[:2])) < 1.0:
                print("Found Goal")    
                sys.exit('Found Goal')
            '''
        
        return closest_vertex


    # ### Selecting Inputs
    # 
    # Select input which moves `x_near` closer to `x_rand`. This should return the angle or orientation of the vehicle.


    def select_input(self, x_rand, x_near):
        return np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])


    # ### New State
    # 
    # 

    # The new vertex `x_new` is calculated by travelling from the current vertex `x_near` with a orientation `u` for time `dt`.


    def new_state(self, x_near, u, dt):
        nx = x_near[0] + np.cos(u)*dt
        ny = x_near[1] + np.sin(u)*dt
        return [nx, ny]


    # ### Putting It All Together
    # 
    # Awesome! Now we'll put everything together and generate an RRT.

    

    def generate_path(grid, h, x_init, x_goal, rrt_new, x_near, rrt):
       
        print("Mapping path from goal to start")

        x_goal = (30, 750)
        
        num_vertices = 1600
        dt = 18
        x_init = (20, 150)
        path = [(20, 30), (40, 50)]

        #self.rrt_goal = round(x_near[0],[1])      
        print ("Goal Found.")
        found = True

        

        #rrt_path = nx.DiGraph()
        #rrt_path.add_node(x_init)
        #parent_node = RRT.parent(self, rrt_new)
    
        found = False

        # find path from goal
        current_node = rrt_new
        p = RRT_path.parent(rrt_new)
        
        if found: 

            while p is not None:
            
                for _ in range(num_vertices):

                    #edge_cost = int(h) 
                    #item = queue.get()
                    #self.branch[edge_cost] = (tuple(rrt_new), x_near) 
                    #parent_node = (self.tree.predecessors(i))
                    
                
                    #parent_node = self.parent
                    RRT_path.rrt_path.add_node(current_node)
                    current_node = RRT_path.parent(current_node)
                    
                    print ("rrt_path", RRT_path.rrt_path)

                    if current_node == x_init:
                        return RRT_path.rrt_path


                plt.imshow(grid, cmap='Greys', origin='lower')
                plt.plot(RRT_path.x_init[1], RRT_path.x_init[0], 'ro')
                plt.plot(RRT_path.x_goal[1], RRT_path.x_goal[0], 'ro')
            
                print ("RRT_path goal", RRT_path.RRT_path_goal)   
                #plt.plot(RRT_path.RRT_path_goal[1], RRT_path.RRT_path_goal[0], 'ro')
                
                #RRT_path = RRT_path.generate_RRT_path(self, grid, RRT_path.x_init, RRT_path.num_vertices, RRT_path.dt)
                for (v1, v2) in RRT_path.RRT_path_path:
                    plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')
                
                plt.show(block=True)


            
                #RRT.memoize_nodes(self, grid, rrt_cost, x_init, x_goal, x_new, x_near, rrt, u)
                return rrt #, self.rrt_goal

        elif grid[int(rrt_new[0]), int(rrt_new[1])] == 0:
            # the orientation `u` will be added as metadata to
            # the edge
            rrt.add_edge(x_near, rrt_new, u)
            #self.memoize_nodes(grid, rrt_cost, x_init, x_goal, rrt_new, x_near, rrt, u)
            #States
            print ("RRT Path Mapped")

            return rrt 


        plt.show(block=True)
    
    def heuristic(position, goal_position):
        return np.linalg.norm(np.array(position) - np.array(goal_position))


         

    
