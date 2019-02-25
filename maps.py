"""
Robotic Simulation 
Map generation

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import numpy as np
import copy
from animation import animation
from a_star import a_star
import matplotlib.pyplot as plt
from RRT import RRT

class maps():
    
    def __init__(self, grid_res, map_limits, obstacles, inflation=0):
        self.x = []
        self.y = []
        self.x_inflated = []
        self.y_inflated = []
        self.resolution = grid_res
        self.min_lim_x = map_limits[0]/self.resolution
        self.max_lim_x = map_limits[1]/self.resolution
        self.min_lim_y = map_limits[2]/self.resolution
        self.max_lim_y = map_limits[3]/self.resolution
        self.inflation = inflation/self.resolution
        
        # limits of the map
        for i in range(int(self.min_lim_x), int(self.max_lim_x)):
            self.x.append(i)
            self.y.append(int(self.min_lim_y))
        for i in range(int(self.min_lim_x), int(self.max_lim_x)):
            self.x.append(i)
            self.y.append(int(self.max_lim_y))
        for i in range(int(self.min_lim_y), int(self.max_lim_y)):
            self.x.append(int(self.min_lim_x))
            self.y.append(i)
        for i in range(int(self.min_lim_y), int(self.max_lim_y)):
            self.x.append(int(self.max_lim_x))
            self.y.append(i)
        
        self.x_inflated = copy.deepcopy(self.x)
        self.y_inflated = copy.deepcopy(self.y)
        
        for obstacle in obstacles:
            self.add_obstacle_square(obstacle[0], obstacle[1], obstacle[2], self.inflation)
            self.add_obstacle_square(obstacle[0], obstacle[1], obstacle[2], 0.0)
        
    # add obstacle center and size
    def add_obstacle_square(self, obstacle_x, obstacle_y, size, inflation):
        size = size/self.resolution
        for i in range(int(obstacle_x/self.resolution - size/2- inflation), int(obstacle_x/self.resolution + size/2 + inflation)):
            for j in range(int(obstacle_y/self.resolution - size/2 - inflation), int(obstacle_y/self.resolution + size/2 + inflation)):            
                if inflation == 0.0:                
                    self.x.append(i)
                    self.y.append(j)
                else:
                    self.x_inflated.append(i)
                    self.y_inflated.append(j)

    # take the positions of the obstacles and limits and make a grid
    def make_grid(self):
        size_x = self.max_lim_x - self.min_lim_x
        size_y = self.max_lim_y - self.min_lim_y
        grid = [[0 for col in range(int(size_y)+1)] for row in range(int(size_x)+1)]
        for i in range(len(self.x_inflated)-1):
            grid[int((self.x_inflated[i]-self.min_lim_x))][int((self.y_inflated[i]-self.min_lim_y))] = 50
        return grid
        
    def plot_map(self, axis):
        animation.plot_map(np.asarray(self.x)*np.asarray(self.resolution), np.asarray(self.y)*np.asarray(self.resolution), axis)
        
        
        
if __name__ == '__main__':
    Astar = a_star()
    resolution = 0.1 # map resolution
    map_limits = [-20, 20, -20, 20] # [min_x, max_x, min_y, max_y]
    obstacles = ([4, 6, 5],
                 [-2, 0, 2])  
#    obstacles = []
    maps = maps(resolution, map_limits, obstacles, 2.25) 
    # get grid map    
    grid = maps.make_grid()   
    fig, axs = plt.subplots()
    maps.plot_map(axs)
    init = [int((-8--10)/resolution), int((-8--20)/resolution)]
    goal = [int((9--10)/resolution), int((15--20)/resolution)]
#    Astar.astar_search(grid,init,goal)
    start = [0,0]
    goal = [10,6]
    rrt = RRT(start, goal, obstacles, map_limits, 0.5, 10, 10000)
    rrt.initialize_RRT()
    
    
    
    
    
    
    
    
    