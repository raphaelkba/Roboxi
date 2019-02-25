"""
Robotic Simulation 
Path planner class

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import matplotlib.pyplot as plt
class Planner():
    def __init__(self, start, goal, map_limits):
        """
        Initialize planner
        """
        self.start = start
        self.goal = goal
        self.min_lim_x = map_limits[0]
        self.max_lim_x = map_limits[1]
        self.min_lim_y = map_limits[2]
        self.max_lim_y = map_limits[3]
        self.planner_animation = True
        self.fig, self.ax = plt.subplots()
        
    def plan(self, *args):
        pass
    
    def find_path(self, *args):
        pass
    
    def animation(self, *args):
        pass
    
    