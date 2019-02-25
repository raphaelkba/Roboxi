"""
Robotic Simulation 

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import matplotlib.pyplot as plt
import numpy as np

from controllers import controllers
from utils import utils
from animation import animation      
from maps import maps
from a_star import a_star
from robots import diff_drive, extended_bicycle, front_wheel_drive
from RRT import RRT

if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    time = 0.0
    dT = 0.01
    anime = animation()
    play_animation = True
    ###################### Initialize map ###################### 
    resolution = 1 # map resolution
    map_limits = [-20, 20, -20, 20] # [min_x, max_x, min_y, max_y]
    obstacles = ([4, 6, 3],
                 [0, 0, 2])    
    inflation = 2.25
    maps = maps(resolution, map_limits, obstacles, inflation) 
    grid = maps.make_grid() # get grid map with inflated obstacles
    
    ###################### Initialize controls and states ###################### 
    control = controllers()
    
    states = np.array([[-8.0],
                       [-8.0],
                      [0*np.pi],[0]])
#                      
#    states = np.array([[-8.0],
#                       [-8.0],
#                      [np.pi*0],
#                      [0.0],
#                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    goal = np.array([10.0, 8.0, 0.0])
        
    
    states_history = states
    controls_history = controls
    
    ###################### Initialize path planning ###################### 
    planner = a_star(resolution, map_limits, states, goal)    
    path = planner.plan(grid)
    
#    rrt = RRT(states, goal, obstacles, map_limits, 0.5, 10, 10000)
#    path = rrt.initialize_RRT()
    
    
    ###################### Initialize model ###################### 
    simple_bicycle_test = front_wheel_drive("simple bicycle", states, controls, path, dT)
    
    print("Start Simulation")
    while utils.euclidean_distance(goal, simple_bicycle_test.states) > 0.1:
        
        simple_bicycle_test.run()        
        states_history = np.hstack((states_history, simple_bicycle_test.states))
        controls_history = np.hstack((controls_history, simple_bicycle_test.controls))
        time += dT

        if play_animation:
            anime.animate(states_history, simple_bicycle_test.goal, path, maps, simple_bicycle_test.controls)         
    
    print("Simulation Finished")
