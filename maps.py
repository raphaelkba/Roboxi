"""
Robotic Simulation 
Map generation

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import math
import numpy as np
from animation import animation
from a_star import a_star

class maps():
    
    def __init__(self, grid_res, min_lim_x, max_lim_x, min_lim_y, max_lim_y):
        self.x = []
        self.y = []
        self.resolution = grid_res
        self.min_lim_x = min_lim_x/self.resolution
        self.max_lim_x = max_lim_x/self.resolution
        self.min_lim_y = min_lim_y/self.resolution
        self.max_lim_y = max_lim_y/self.resolution   
        
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
    
    # add obstacle center and size
    def add_obstacle_square(self, obstacle_x, obstacle_y, size):
        size = size/self.resolution
        for i in range(int(obstacle_x/self.resolution - size/2), int(obstacle_x/self.resolution + size/2)):
            for j in range(int(obstacle_y/self.resolution - size/2), int(obstacle_y/self.resolution + size/2)):            
                self.x.append(i)
                self.y.append(j)
    
    # take the positions of the obstacles and limits and make a grid
    def make_grid(self):
        size_x = self.max_lim_x - self.min_lim_x
        size_y = self.max_lim_y - self.min_lim_y
        grid = [[0 for col in range(int(size_y)+1)] for row in range(int(size_x)+1)]
        for i in range(len(self.x)-1):
            grid[int((self.x[i]-self.min_lim_x))][int((self.y[i]-self.min_lim_y))] = 1
        return grid
        
    def plot_map(self, axis):
        animation.plot_map(np.asarray(self.x)*np.asarray(self.resolution), np.asarray(self.y)*np.asarray(self.resolution), axis)
        
        
        
if __name__ == '__main__':
    Astar = a_star()
    resolution = 0.1
    grid_map = maps(resolution, -10, 10, -20, 20)    
    grid_map.add_limits(10,10)
    grid_map.add_obstacle_square(3,12,5)
    grid_map.add_obstacle_square(0,0,8)
    grid = grid_map.make_grid(100,100)
    grid_map.plot_map()
    init = [int((-8--10)/resolution), int((-8--20)/resolution)]
    goal = [int((9--10)/resolution), int((15--20)/resolution)]
    Astar.astar_search(grid,init,goal)
    
    
    