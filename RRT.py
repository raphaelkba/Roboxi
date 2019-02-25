"""
Robotic Simulation 
RRT path planning
Source: http://msl.cs.uiuc.edu/~lavalle/papers/LavKuf01.pdf
Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import math
import random
import numpy as np
import matplotlib.pyplot as plt
from utils import utils
from planner import Planner
import matplotlib.patches as mpatches
from controllers import controllers
from robots import diff_drive, extended_bicycle, front_wheel_drive, simple_bicycle


class RRT(Planner):
    
    def __init__(self, start, goal, obstacles, 
                 map_limits, expand_distance, max_itr):
                     
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.expand_distance = expand_distance
        self.max_itr = max_itr
        self.nodes = []
        self.nodes.append([self.start[0], self.start[1], -1])
        self.gains = np.array([0.1, 0.0, 1.0, -0.5])
        self.control = controllers(self.gains)        
        super().__init__(self.start, self.goal, map_limits)

        
    def plan(self):
        print("Start RRT")
        itr = 0
        while itr < self.max_itr:
            print(itr)
            
            if random.randint(0, 100) > 10:
                x_rand = [random.uniform(self.min_lim_x, self.max_lim_x), random.uniform(
                    self.min_lim_y, self.max_lim_y)]
            else:
                x_rand = [self.goal[0], self.goal[1]]
                    
            closest_node, closest_idx = utils.find_closest_point(self.nodes, x_rand)    
        
            new_node = self.extend(x_rand, closest_node, closest_idx)
        
            if not self.check_collision(new_node):
                continue

            self.nodes.append(new_node)            
            if self.check_goal(new_node):
                print("RRT found a goal")
                break
            itr += 1
        self.animation()
        
        path = self.find_path()
        path_x = np.array(path[:,0])
        path_y = np.array(path[:,1])
        path = [path_x[::-1], path_y[::-1]]

        return path
        
    def extend(self, xrand, closest_node, closest_idx):
        orientation = math.atan2(xrand[1] - closest_node[1], xrand[0] - closest_node[0]) 
        new_node = [0,0,0]
        new_node[0] = closest_node[0] + self.expand_distance*math.cos(orientation)
        new_node[1] = closest_node[1] + self.expand_distance*math.sin(orientation)
        new_node[2] = closest_idx 
        return new_node                     
                           
    def check_collision(self, node):
        for obstacle in self.obstacles:
            if (math.sqrt((obstacle[0] - node[0])*(obstacle[0] - node[0]) + (obstacle[1] - node[1])*(obstacle[1] - node[1]))) < obstacle[2]:
                return False
        return True
    
    def check_goal(self, node):
        if ((self.goal[0] - node[0])** 2 + (self.goal[1] - node[1])** 2) < self.expand_distance:
            return True
    
    def find_path(self):
        path = [[self.goal[0], self.goal[1]]]
        idx = self.nodes[-1][2]
        while self.nodes[idx][2] != -1:
            path.append([self.nodes[idx][0][0], self.nodes[idx][1][0]])
            idx = self.nodes[idx][2]
        path.append([self.start[0][0], self.start[1][0]])
        return np.array(path)
        
    def animation(self):
        for obstacle in self.obstacles:
            obs = mpatches.FancyBboxPatch((obstacle[0]-obstacle[2]/2, obstacle[1]-obstacle[2]/2), obstacle[2], obstacle[2], boxstyle=mpatches.BoxStyle("Round", pad=0.01),color='red')  
            self.ax.add_patch(obs)   
            
        self.ax.plot(self.goal[0], self.goal[1], "xr")            
        self.ax.plot(self.start[0], self.start[1], "ob")
        
        for node in self.nodes:
            if node[2] != -1:
                plt.plot([node[0], self.nodes[node[2]][0]], [node[1], self.nodes[node[2]][1]], "-k")
        
        self.ax.axis('equal')
        plt.pause(0.001)
