"""
Robotic Simulation 

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import matplotlib.pyplot as plt
import numpy as np

from models import models
from controllers import controllers
from utils import utils
from animation import animation      
from maps import maps
from a_star import a_star
from robots import robots
from RRT import RRT

if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    time = 0.0
    dT = 0.1
    anime = animation()
    ###################### Initialize map ###################### 
    resolution = 0.1 # map resolution
    map_limits = [-20, 20, -20, 20] # [min_x, max_x, min_y, max_y]
    obstacles = ([4, 6, 5],
                 [0, 0, 2])    
    maps = maps(resolution, map_limits, obstacles, 2.25) 
    grid = maps.make_grid() # get grid map with inflated obstacles
    
    ###################### Initialize controls and states ###################### 
    control = controllers()
    
    states = np.array([[-8.0],
                       [-8.0],
                      [0.0]])#,
                     # [0.0],
#                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    goal = np.array([15.0, 17.0, 0.0])
        
    ###################### Choose model ###################### 
    robot = models("simple bicycle", states, dT)
    #robot = models("front wheel bicycle", states, dT)
#    control_gain = np.array([1.0, 100.0, 0.0, 1.0, -0.0])
    states_history = states
    controls_history = controls
    
    ###################### Initialize path planning ###################### 
    planner = a_star()    
    # grid map begins at [0,0]
    init = [int((states[0] - map_limits[0])/resolution), int((states[1] - map_limits[2])/resolution)]
    end = [int((goal[0] - map_limits[0])/resolution), int((goal[1] - map_limits[2])/resolution)]
    
    path, action_map = planner.astar_search(grid,init,end)
    
    rrt = RRT(states, goal, obstacles, map_limits, 1.0, 10, 10000)
    path = rrt.initialize_RRT()
    
    # readjust to normal coordinater
#    path_x = np.array(path[:,0]*resolution + map_limits[0])
#    path_y = np.array(path[:,1]*resolution + map_limits[2])
    path_x = np.array(path[:,0])
    path_y = np.array(path[:,1])
    path_ = [path_x[::-1], path_y[::-1]]
    path = path_
    
    simple_bicycle = robots("simple bicycle", states, controls, path, dT)    
    
    print("Start Simulation")
    while utils.euclidean_distance(goal, states) > 0.1:
        
        simple_bicycle.run()

        #pid = control.LQR_Bicycle(states, goal, dT)
        #pid[1] = (pid[1] - states[2][0])/dT # front wheel drive
        
        states_history = np.hstack((states_history, simple_bicycle.states))
        controls_history = np.hstack((controls_history, simple_bicycle.controls))
        time += dT

        if play_animation:
            anime.animate(states_history, simple_bicycle.goal, path, maps, simple_bicycle.controls)         
    
    print("Simulation Finished")
