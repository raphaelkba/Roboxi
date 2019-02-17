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



if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    play_animation = True    
    time = 0.0
    dT = 0.1
    anime = animation()
    lookahead_idx = 10
    ######## Initialize map ########
    resolution = 0.1 # map resolution
    min_x = -10 # minimum map limit x axis
    max_x = 10 # maximum map limit x axis
    min_y = -20 # minimum map limit y axis   
    max_y = 20 # maximum map limit y axis
    maps = maps(resolution, min_x, max_x, min_y, max_y) 
    # add obstacles    
    maps.add_obstacle_square(3,12,1)
    maps.add_obstacle_square(0,0,2)
    # get grid map    
    grid = maps.make_grid()
    
    ######## Initialize controls and model ########
    control = controllers()
    
    states = np.array([[-8.0],
                       [-8.0],
                      [0.0],
                      [0.0]])
#                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    
    goal = np.array([5.0, 2.0, 0.0])
    
    goal_final = np.array([5.0, 2.0, 0.0])
    
    ######## Choose model ########
#    robot = models("simple bicycle", states, dT)
    robot = models("front wheel bicycle", states, dT)
    control_gain = np.array([1.0, 100.0, 0.0, 1.0, -0.0])
    control_gain = np.array([1.0, 0.0, 0.0, 2.0, -1.0])
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
    
    print("Start Simulation")
    while utils.euclidean_distance(goal_final, states) > 0.1:# and (goal_final[2] - states[2]) < 0.01):

        path_, goal = control.look_ahead(path_, goal_final, states, lookahead_idx)

        pid = control.simple_control(control_gain, states, goal)
        #pid = control.LQR_Bicycle(states, goal, dT)
        pid[1] = (pid[1] - states[2][0])/dT
        states = robot.runge_kutta_solver(states, dT, dT/10, pid)
        
        
        states[2][0] = utils.truncate_angle(states[2][0])


        max_df = math.pi/3
        min_df = -math.pi/3
#        if states[3][0] < -50.0:
#            states[3][0] = -50.0
#        elif states[3][0] > 50.0:
#            states[3][0] = 50.0
        if states[3][0] < min_df:
            states[3][0] = min_df
        elif states[3][0] > max_df:
            states[3][0] = max_df
        print(states)
        
        
        states_history = np.hstack((states_history, states))
        

#        time += dT


        if play_animation:
            anime.animate(states_history, goal, path, maps, pid)         
    print("Simulation Finished")
