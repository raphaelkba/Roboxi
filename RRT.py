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

class RRT():
    
    def __init__(self, start, goal, obstacles, 
                 maps_limits, expandDist, goalSamplesRate, maxIter):
                     
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.expandDist = expandDist
        self.goalSamplesRate = goalSamplesRate
        self.maxIter = maxIter
        self.nodes = []
        self.nodes.append([self.start[0], self.start[1], -1])
        self.min_lim_x = maps_limits[0]
        self.max_lim_x = maps_limits[1]
        self.min_lim_y = maps_limits[2]
        self.max_lim_y = maps_limits[3]

        
    def initialize_RRT(self):
        print("Start RRT")
        while True:
            x_rand = [random.uniform(self.min_lim_x, self.max_lim_x), random.uniform(
                    self.min_lim_y, self.max_lim_y)]
                    
            closest_node, closest_idx = utils.find_closest_point(self.nodes, x_rand)    
        
            new_node = self.extend(x_rand, closest_node, closest_idx)
        
            if not self.check_collision(new_node):
                continue

            self.nodes.append(new_node)            
            if self.check_goal(new_node):
                print("RRT found a goal")
                break
        
        path = self.find_path()
#        self.DrawGraph()
#        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
#        plt.pause(0.01)
#        plt.show()
        return np.array(path)
        
    def extend(self, xrand, closest_node, closest_idx):
        orientation = math.atan2(xrand[1] - closest_node[1], xrand[0] - closest_node[0]) 
        new_node = [0,0,0]
        new_node[0] = closest_node[0] + self.expandDist*math.cos(orientation)
        new_node[1] = closest_node[1] + self.expandDist*math.sin(orientation)
        new_node[2] = closest_idx 
        return new_node        
        
    
    def check_collision(self, node):
        for obstacle in self.obstacles:
            if ((obstacle[0] - node[0])*(obstacle[0] - node[0]) + (obstacle[1] - node[1])*(obstacle[1] - node[1])) < 4.5 + math.sqrt(2*obstacle[2]*obstacle[2]):
                return False
        return True
    
    def check_goal(self, node):
        if ((self.goal[0] - node[0])** 2 + (self.goal[1] - node[1])** 2) < self.expandDist:
            return True
    
    def find_path(self):
        path = [[self.goal[0], self.goal[1]]]
        idx = self.nodes[-1][2]
        while self.nodes[idx][2] != -1:
            path.append([self.nodes[idx][0][0], self.nodes[idx][1][0]])
            idx = self.nodes[idx][2]
        path.append([self.start[0][0], self.start[1][0]])
        return path        
        
    def DrawGraph(self, rnd=None):  # pragma: no cover
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodes:
            if node[2] is not -1:
                plt.plot([node[0], self.nodes[node[2]][0]],
                         [node[1], self.nodes[node[2]][1]], "-g")

        for (ox, oy, size) in self.obstacles:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)