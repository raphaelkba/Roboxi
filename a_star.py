"""
Robotic Simulation 
A Star path planning

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import math
import numpy as np
import matplotlib.pyplot as plt

def calculate_heuristic(pos, goal):
    w = 1.0  # weight of heuristic
    d = w * math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
    return d      


class a_star:
        
    def __init__(self):#, grid, init, goal, cost):
        pass
        #self.grid = grid
        #self.initial_position = init
        #self.goal_position = goal
        #self.cost = cost     
        

        
    def astar_search(self, grid,init,goal):
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

        closed[init[0]][init[1]] = 1
    
        expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
        action = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    
        x = init[0]
        y = init[1]
        g = 0
        f = 0
    
        open = [[f, g, x, y]]
    
        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0
        
        while not found and not resign:
            if len(open) == 0:
                resign = True
                return "Fail"
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[3]
                g = next[1]
                expand[x][y] = count
                count += 1
                
                if x == goal[0] and y == goal[1]:
                    found = True
                else:
                    for i in range(len(possible_movements)):
                        x2 = x + possible_movements[i][0]
                        y2 = y + possible_movements[i][1]
                        if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                            if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                                g2 = g + possible_movements[i][2] 
                                f = g2 + calculate_heuristic([x2,y2],goal)
                                open.append([f, g2, x2, y2])
                                closed[x2][y2] = 1
                                deltas[x2][y2] = i
        path = []
        path.append([goal[0],goal[1]])
        x = goal[0]
        y = goal[1]
        idx = 1
        while [x,y] != [init[0],init[1]]:
            x_ = x - possible_movements[deltas[x][y]][0]
            y_ = y - possible_movements[deltas[x][y]][1]
            path.append([x_, y_])            
            action[x_][y_] = idx
            x = x_ 
            y = y_ 
            idx += 1
        
        plt.matshow(np.rot90(np.array(action)))      
        path = np.array(path)               
        return path, action
        
