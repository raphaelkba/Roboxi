"""
Robotic Simulation 
A Star path planning

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import copy
from planner import Planner

def calculate_heuristic(pos, goal):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
    return d      


class a_star(Planner):
        
    def __init__(self, resolution, map_limits, states, goal):#, grid, init, goal, cost):
        self.resolution = resolution
        # grid map begins at [0,0]
        self.init = [int((states[0] - map_limits[0])/resolution), int((states[1] - map_limits[2])/resolution)]
        self.goal = [int((goal[0] - map_limits[0])/resolution), int((goal[1] - map_limits[2])/resolution)]
        super().__init__(self.init, self.goal, map_limits)
        
        
    def plan(self, grid):
        # x,y, cost
        possible_movements =[[-1,  0, 1], # go up
                            [-1,  -1, math.sqrt(2)], # up left 
                            [ 0, -1, 1], # go left
                            [ 1, -1, math.sqrt(2)], # down left
                            [ 1,  0, 1], # go down
                            [ 1,  1, math.sqrt(2)], # down right
                            [ 0,  1, 1], # go right
                            [ -1, 1, math.sqrt(2)]]# up right
        
        closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
        deltas = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
        grid_plot = copy.deepcopy(grid)
        closed[self.init[0]][self.init[1]] = 1
    
        action = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    
        x = self.init[0]
        y = self.init[1]
        g = 0
        f = 0
    
        open_list = [[f, g, x, y]]
    
        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0
        
        while not found and not resign:
            if len(open_list) == 0:
                resign = True
                return "Fail"
            else:
                open_list.sort()
                open_list.reverse()
                next_item = open_list.pop()
                x = next_item[2]
                y = next_item[3]
                g = next_item[1]
                count += 1
                
                if x == self.goal[0] and y == self.goal[1]:
                    found = True
                else:
                    for i in range(len(possible_movements)):
                        x2 = x + possible_movements[i][0]
                        y2 = y + possible_movements[i][1]
                        if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                            if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                                g2 = g + possible_movements[i][2] 
                                f = g2 + calculate_heuristic([x2,y2],self.goal)
                                open_list.append([f, g2, x2, y2])
                                closed[x2][y2] = 1
                                deltas[x2][y2] = i
                                grid_plot[x2][y2] = 1
                                
                self.animation(open_list, next_item, grid_plot)
                
        path = self.find_path(possible_movements, deltas, action, grid_plot)
        
        return path
        
    def find_path(self, possible_movements, deltas, action, grid_plot):
        path = []
        path.append([self.goal[0],self.goal[1]])
        x = self.goal[0]
        y = self.goal[1]
        idx = 1
        while [x,y] != [self.init[0],self.init[1]]:
            x_ = x - possible_movements[deltas[x][y]][0]
            y_ = y - possible_movements[deltas[x][y]][1]
            path.append([x_, y_])            
            action[x_][y_] = idx
            x = x_ 
            y = y_ 
            idx += 1
        
        
        path = np.array(path) 
        for i in reversed(range(len(path))):            
            grid_plot[path[i][0]][path[i][1]] = 20+i
            self.ax.matshow(np.rot90(np.array(grid_plot)))      
            plt.pause(0.001)   
        path_x = path[:,0]*self.resolution + self.min_lim_x
        path_y = path[:,1]*self.resolution + self.min_lim_y 
        path = [path_x[::-1], path_y[::-1]]
     
        
        return path
        
    def animation(self, open_list, next_item, grid_plot):

        for item in open_list:
            grid_plot[item[2]][item[3]] = 1   
        grid_plot[next_item[2]][next_item[3]] = 5
        grid_plot[self.goal[0]][self.goal[1]] = 10
        grid_plot[self.init[0]][self.init[1]] = 15
        self.ax.matshow(np.rot90(np.array(grid_plot))) 
        plt.pause(0.001)
        