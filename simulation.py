"""
Roboxi - Simulation 

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""
import matplotlib.pyplot as plt
import numpy as np
import copy

from controllers import controllers
from utils import utils
from animation import animation      
from maps import maps
from a_star import a_star
from robots import diff_drive, extended_bicycle, front_wheel_drive, simple_bicycle
from RRT import RRT
from kalman_filter import Kalman_filter
from manual_control import ManualControl


import cubic_spline_planner

if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    time = 0.0
    dT = 0.1
    anime = animation()
    play_animation = True
    ###################### Initialize map ###################### 
    resolution = 0.1 # map resolution
    map_limits = [-20, 20, -20, 20] # [min_x, max_x, min_y, max_y]
    obstacles = ([2, 8, 5],
                 [-2, 3, 6],
                 [9, -4, 7]) 
#    obstacles = ()
    inflation = 3.0
    maps = maps(resolution, map_limits, obstacles, inflation) 
    grid = maps.make_grid() # get grid map with inflated obstacles
    
    ###################### Initialize controls and states ###################### 
    
#    states = np.array([[-8.0],
#                       [-8.0],
#                      [0*np.pi],[0]])
#                      
#    states = np.array([[-8.0],
#                       [-8.0],
#                      [0*np.pi]])
                      
    states = np.array([[-8.0],
                        [-8.0],
                      [np.pi*0],
                      [0.0],
                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    goal = np.array([15.0, 15.0, 0.0])
        
    
    states_history = states
    states_history_ground_truth = states
    controls_history = controls
    
    ###################### Initialize path planning ###################### 
    planner = a_star(resolution, map_limits, states, goal)    
    path = planner.plan(grid)
    
#    rrt = RRT(states, goal, obstacles, map_limits, 2.0, 10000)
#    path = rrt.plan()
    
    rx, ry, ryaw, rk, s = cubic_spline_planner.calc_spline_course(path[0], path[1], ds=0.5)
    path = [rx, ry]
    ################ Initialize model and control ################# 
    robot_ground_truth = extended_bicycle("RK2", states, controls, path, dT)
    robot = extended_bicycle("RK3", states, controls, path, dT)
        
    control = controllers(robot.gains)    
    
    ekf = Kalman_filter(states.shape[0])
    mc = ManualControl()
    
    
#    def on_key(event):
#        print('you pressed', event.key, event.xdata, event.ydata)


    print("Start Simulation")
    while utils.euclidean_distance(goal, robot.states) > 0.3:
        
#        cid = anime.fig.canvas.mpl_connect('key_press_event', on_key)
        
#        robot.controls = mc.get_controls(anime.fig, robot.states, dT)
        robot.controls = control.lqr_vel_steer_control(robot)
        robot_ground_truth.controls = copy.deepcopy(robot.controls)
#        
        add_noise = False
        robot_ground_truth.run(add_noise) # ground truth
##        add_noise = False
        robot.run(add_noise)
##        robot = ekf.run_filter(robot, robot_ground_truth.states) 
#
        print(robot.states)
        states_history = np.hstack((states_history, robot.states))
        states_history_ground_truth = np.hstack((states_history_ground_truth, robot_ground_truth.states))
        controls_history = np.hstack((controls_history, robot.controls))
        time += dT
#        plt.pause(0.25)
        if play_animation:
            anime.animate(states_history, robot_ground_truth, path , maps)         
            
    print("Simulation Finished")

