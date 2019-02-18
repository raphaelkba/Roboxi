"""
Robotic Simulation 

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import matplotlib.pyplot as plt
import numpy as np
import math

from models import models
from controllers import controllers
from utils import utils
from animation import animation      
from maps import maps
from a_star import a_star
from robots import robots


if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    play_animation = True    
    time = 0.0
    dT = 0.1
    anime = animation()
    lookahead_idx = 15
    ######## Initialize map ########
    resolution = 0.1 # map resolution
    min_x = -20 # minimum map limit x axis
    max_x = 20 # maximum map limit x axis
    min_y = -20 # minimum map limit y axis   
    max_y = 20 # maximum map limit y axis
    maps = maps(resolution, min_x, max_x, min_y, max_y) 
    # add obstacles    
    maps.add_obstacle_square(4,6,5)
    maps.add_obstacle_square(-2,0,2)
    # get grid map    
    grid = maps.make_grid()
    
    ######## Initialize controls and model ########
    control = controllers()
    
    states = np.array([[-8.0],
                       [-8.0],
                      [0.0]])#,
                     # [0.0],
#                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    
    goal = np.array([15.0, 17.0, 0.0])
    
    goal_final = np.array([15.0, 17.0, 0.0])
    
    ######## Choose model ########
    robot = models("simple bicycle", states, dT)
    #robot = models("front wheel bicycle", states, dT)
#    control_gain = np.array([1.0, 100.0, 0.0, 1.0, -0.0])
    states_history = states
    controls_history = controls
    
    ######## Initialize path planning ########    
    planner = a_star()    
    # grid map begins at [0,0]
    init = [int((states[0] - min_x)/resolution), int((states[1] - min_y)/resolution)]
    end = [int((goal[0] - min_x)/resolution), int((goal[1] - min_y)/resolution)]
    
    path, action_map = planner.astar_search(grid,init,end)
    
    # readjust to normal coordinater
    path_x = np.array(path[:,0]*resolution + min_x)
    path_y = np.array(path[:,1]*resolution + min_y)
    path_ = [path_x[::-1], path_y[::-1]]
    path = path_
    
    simple_bicycle = robots("simple bicycle", states, controls, path_, dT)    
    
    print("Start Simulation")
    while utils.euclidean_distance(goal_final, states) > 0.1:# and (goal_final[2] - states[2]) < 0.01):

        simple_bicycle.run()

        #pid = control.LQR_Bicycle(states, goal, dT)
        #pid[1] = (pid[1] - states[2][0])/dT # front wheel drive
        
        states_history = np.hstack((states_history, simple_bicycle.states))
        controls_history = np.hstack((controls_history, simple_bicycle.controls))
        time += dT


        if play_animation:
            anime.animate(states_history, simple_bicycle.goal, path, maps, simple_bicycle.controls)         
    print("Simulation Finished")
