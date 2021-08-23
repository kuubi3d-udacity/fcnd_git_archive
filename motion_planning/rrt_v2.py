import sys

import argparse
import time
from joblib import MemorizedResult
import msgpack
from enum import Enum, auto

import numpy as np
import decimal


# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from operator import itemgetter

from sympy import memoize_property
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
     

    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

        self.rrt_path = nx.DiGraph()
        self.rrt_path.add_node(x_init)
                
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
        self.rrt_path.add_node(tuple(RRT.x_init))
    
    def add_rrt_edge(self, x_near, x_new, u):
        self.rrt_path.add_edge(tuple(x_near), tuple(x_new), orientation=u)

    @property
    def rrt_vertices(self):
        return self.rrt_path.nodes()
   
    @property
    def rrt_edges(self):
        return self.rrt_path.edges()    

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

    

    def generate_RRT(self, grid, x_init, num_vertices, dt,):
       
        
        x_goal = (30, 750)
        #rrt_goal = ()
        num_vertices = 1600
        dt = 18
        x_init = (20, 150)
        #path = [(20, 30), (40, 50)]
        goal_path = []
        path_cost = 0

        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        
        # TODO: set home position to (lon0, lat0, 0)

        # TODO: retrieve current global position
 
        # TODO: convert to current local position using global_to_local()
        
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid, north_offset, east_offset = RRT.create_grid(self, data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        
         # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        print ('Generating RRT. It may take a few seconds...')
        rrt = RRT(x_init)

        for _ in range(num_vertices):
                      
            x_rand = RRT.sample_state(self, grid)
            # sample states until a free state is found
            while grid[int(x_rand[0]), int(x_rand[1])] == 1:
                x_rand = RRT.sample_state(self, grid)
                                  
            x_near = RRT.nearest_neighbor(self, x_rand, rrt)
            u = RRT.select_input(self, x_rand, x_near)
            x_new = RRT.new_state(self, x_near, u, dt)

            RRT.memoize_nodes(grid, h, grid_start, grid_goal, goal_path, path_cost)
            
            v_near = np.array([30, 750])
            norm_g = np.array(x_goal)
            norm_n = np.array(x_near)
            #norm_n = np.array(v_near)
            
            print (norm_g, norm_n)
            print (np.linalg.norm(norm_g - norm_n))

            if np.linalg.norm(norm_g - norm_n) < 200:
               rrt.add_edge(x_near, x_new, u)
               
               #self.rrt_goal = round(x_near[0],[1])      
               print ("Goal Found.")
               return rrt #, self.rrt_goal

            elif grid[int(x_new[0]), int(x_new[1])] == 0:
                # the orientation `u` will be added as metadata to
                # the edge
                rrt.add_edge(x_near, x_new, u)
            
            print ("RRT Path Mapped")

            return rrt 
        
    def memoize_nodes(grid, h, grid_start, grid_goal, goal_path, path_cost):
        """
        Given a grid and heuristic function returns
        the lowest cost path from start to goal.
        """
        print("memoizing nodes", "\n")
              

        
        queue = PriorityQueue()
        queue.put((0, grid_start))
        visited = set(grid_start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            
            current_node = item[0]
            current_cost = item[1]

            if current_node == grid_goal:
                print('Found a path.')
                found = True
                break
            else:
                # Get the new vertexes connected to the current vertex
                for a in RRT.vertices(grid, current_node):
                    next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                    new_cost = current_cost + a.cost + h(next_node, grid_goal)

                    if next_node not in visited:
                        visited.add(next_node)
                        queue.put((new_cost, next_node))

                        branch[next_node] = (new_cost, current_node, a)

        if found:
            # retrace steps
            n = grid_goal
            path_cost = branch[n][1]
            goal_path.append(grid_goal)
            while branch[n][0] != grid_start:
                goal_path.append(branch[n][0])
                n = branch[n][1]
            goal_path.append(branch[n][0])
            
            RRT.memoize_nodes()
        else:
            print('**********************')
            print('Failed to find a path!')
            print('**********************')
        return goal_path[::-1], path_cost        
            
    def heuristic(position, goal_position):
        return np.linalg.norm(np.array(position) - np.array(goal_position))       

                   
class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        
        # TODO: set home position to (lon0, lat0, 0)

        # TODO: retrieve current global position
 
        # TODO: convert to current local position using global_to_local()
        
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        grid, north_offset, east_offset = RRT.create_grid(self, data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
       
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        
        self.local_position_callback
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        
        
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        
        rrt = RRT.generate_RRT(self, grid, RRT.x_init, RRT.num_vertices, RRT.dt)
      

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
        
        
        #sys.exit('generating waypoints')
        
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("a_star nodes", path, "\n")
               
        print("rrt nodes", list(rrt.vertices)) #, rrt.edges
        #rrt_path, _= list(rrt.vertices)
         

        #print (RRT.vertices)
        # Convert path to waypoints

        go_path = []
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

        waypoints = [[r[0] + north_offset, r[1] + east_offset, TARGET_ALTITUDE, 0] for r in go_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=240)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
