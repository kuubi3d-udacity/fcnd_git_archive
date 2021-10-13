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

import matplotlib.pyplot as plt
from networkx import Graph
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



class RRT:

    x_goal = (30, 750)
    rrt_goal = ()
    num_vertices = 1600
    dt = 18
    x_init = (20, 150)
    path = [(20, 30), (40, 50)]
     
    path_cost = 0
    g = graphviz.Digraph('RRT Path', format = 'svg', filename='hello.gv')

    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

        self.path_tree = nx.DiGraph()
        self.path_tree.add_node(x_init)

        
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

    
    def add_rrt_vertex(self, x_new):
        self.path_tree.add_node(tuple(RRT.x_init))
    
    def add_rrt_edge(self, x_near, x_new, u):
        self.path_tree.add_edge(tuple(x_near), tuple(x_new), orientation=u)

    @property
    def rrt_vertices(self):
        return self.rrt_path.nodes()
   
    @property
    def rrt_edges(self):
        return self.rrt_path.edges()

    @property
    def parent(self, x_new):
        return self.rrt_path.predecessors(x_new)

    def gview(self):
        return g.view()

    def get_parent(self, x_new):
        return self.tree.predecessors(x_new)


    def create_grid(self, data, drone_altitude, safety_distance):
        """
        Returns a grid representation of a 2D configuration space
        based on given obstacle data, drone altitude and safety distance
        arguments.
        """
   
        # minimum and maximum north coordinates
        north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
        north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

        # minimum and maximum east coordinates
        east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
        east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

        # given the minimum and maximum coordinates we can
        # calculate the size of the grid.
        north_size = int(np.ceil(north_max - north_min))
        east_size = int(np.ceil(east_max - east_min))

        # Initialize an empty grid
        grid = np.zeros((north_size, east_size))

        # Populate the grid with obstacles
        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]
            if alt + d_alt + safety_distance > drone_altitude:
                obstacle = [
                    int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                    int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                    int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                    int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
                ]
                grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
        
        # ~print('INFO', grid, drone_altitude, safety_distance)
        # ~print(grid, int(north_min), int(east_min))        
    

        #print(grid, drone_altitude, safety_distance)
        #print(grid, int(north_min), int(east_min))
        return grid, int(north_min), int(east_min)
    
    def sample_state(self, grid):
        x = np.random.uniform(0, grid.shape[0])
        y = np.random.uniform(0, grid.shape[1])
        return (x, y)


    # ### Nearest Neighbors
    # 
    # A critical part of the RRT procedure is finding the closest vertex to the sampled random point. This the most computationally intensive part so be mindful of that. Depending on the number of vertices a naive implementation will run into trouble quickly.


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
        return (nx, ny)


    # ### Putting It All Together
    # 
    # Awesome! Now we'll put everything together and generate an RRT.

    

    def generate_RRT(self, grid, x_init, num_vertices, current_node, parent_node, dt):
       
        
        x_goal = (30, 750)
        
        num_vertices = 1600
        dt = 18
        x_init = (20, 150)
        path = [(20, 30), (40, 50)]
        

        print ('Planning RRT path. It may take a few seconds...')
        rrt = RRT(x_init)
        rrt_path = RRT(x_init)
        

        for _ in range(num_vertices):

           
            
            x_rand = RRT.sample_state(self, grid)
            # sample states until a free state is found
            while grid[int(x_rand[0]), int(x_rand[1])] == 1:
                x_rand = RRT.sample_state(self, grid)
                                  
            x_near = RRT.nearest_neighbor(self, x_rand, rrt)
            u = RRT.select_input(self, x_rand, x_near)
            x_new = RRT.new_state(self, x_near, u, dt)
            
            #v_near = np.array([30, 750])
            norm_g = np.array(x_goal)
            norm_n = np.array(x_near)
            #norm_n = np.array(v_near)
           
            
            print(norm_g, norm_n)
            print(np.linalg.norm(norm_g - norm_n))
            
            rrt_cost = np.linalg.norm(np.array(x_new) - np.array(x_goal))
            #rrt_cost = np.linalg.norm(norm_g - norm_n)
            print("edge cost", rrt_cost)


            if np.linalg.norm(norm_g - norm_n) < 200:

                print ("Goal Found.")
                rrt.add_edge(x_near, x_new, u)
                current_node = x_new
                #pos = nx.spring_layout(rrt)

                #nx.draw_networkx_nodes(rrt, pos)
                #nx.draw_networkx_labels(rrt, pos)
                #nx.draw_networkx_edges(rrt, pos, edge_color='r', arrows = True)

                #plt.show(block=True)
                #print("rrt path", rrt([0],[1],[2]))

                for _ in range(num_vertices):

                    parent = list(rrt.get_parent(current_node))
                    parent_node = tuple(parent[0])

                    rrt_path.add_rrt_edge(current_node, parent_node, u)
                    print("current_node", current_node)
                    print("parent", parent)
                    print("parent node", parent_node)

                    current_node = parent_node
                    print("new parent", current_node)
                    if parent_node == x_init:
                        print("Path Mapped")

                        plt.imshow(grid, cmap='Greys', origin='lower')
                        plt.plot(RRT.x_init[1], RRT.x_init[0], 'ro')
                        plt.plot(RRT.x_goal[1], RRT.x_goal[0], 'ro')
                    
                        print ("rrt goal", RRT.rrt_goal)   
                        #plt.plot(RRT.rrt_goal[1], RRT.rrt_goal[0], 'ro')

                        for (v1, v2) in rrt_path.path_tree.edges:
                            plt.plot([v1[1], v2[1]], [v1[0], v2[0]], 'y-')
                        
                        plt.show(block=True)
        
                        return rrt, rrt_path

                    #print("rrt path")
                    
                    
                    #memoize_nodes(grid, rrt_cost, x_init, x_goal, current_node, parent_node, rrt, u)
                return rrt, rrt_path 

            elif grid[int(x_new[0]), int(x_new[1])] == 0:
                # the orientation `u` will be added as metadata to
                # the edge
                rrt.add_edge(x_near, x_new, u)
        
        print("RRT Path Mapped")

        return rrt, rrt_path, current_node, parent_node


    def heuristic(position, goal_position):
        return np.linalg.norm(np.array(position) - np.array(goal_position))

