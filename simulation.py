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
from RRT_star import RRT_star
from kalman_filter import Kalman_filter
from manual_control import ManualControl
from mpc import NonlinearMPC


import cubic_spline_planner

if __name__ == '__main__':
    # Initialize simulation parameters
    plt.close("all")
    time = 0.0
    dT = 0.1
    anime = animation()
    play_animation = True
    ###################### Initialize map ########################
    resolution = 0.1 # map resolution
    map_limits = [-20, 20, -20, 20] # [min_x, max_x, min_y, max_y]
    obstacles = ([2, 8, 5],
                 [-2, 3, 6],
                 [9, -4, 7]) 
#    obstacles = () # empty map
    inflation = 3.0
    maps = maps(resolution, map_limits, obstacles, inflation) 
    grid = maps.make_grid() # get grid map with inflated obstacles
    
    ############### Initialize controls and states ############### 
    x = np.random.randint(-20,0) # todo  colission check for intial states
    y = np.random.randint(-20,0)
    theta = np.random.randint(-np.pi/2*10.0,np.pi/2*10.0)/10.0
#    states = np.array([[x],
#                       [y],
#                      [theta],[0]])
#                      
#    states = np.array([[x],
#                       [y],
#                      [theta]])
                      
    states = np.array([[x],
                        [y],
                      [theta],
                      [0.0],
                      [0.0]])
    
    controls = np.array([0.0, 0.0])
    goal = np.array([np.random.randint(0,20), np.random.randint(0,20), 0.0])
        
    
    states_history = states
    states_history_ground_truth = states
    controls_history = controls
    
    ###################### Initialize path planning ###################### 
    planner = a_star(resolution, map_limits, states, goal)    
    path = planner.plan(grid)
    
#    rrt = RRT(states, goal, obstacles, map_limits, 2.0, 10000)
#    path = rrt.plan()
    
#    rrt = RRT_star(states, goal, obstacles, map_limits, 1.0, 500)
#    path = rrt.plan()
    
    # smooth path
    rx, ry, ryaw, rk, s = cubic_spline_planner.calc_spline_course(path[0], path[1], ds=0.25)
    path = [rx, ry]
    
    ################ Initialize model and control ################# 
    robot_ground_truth = extended_bicycle("RK2", states, controls, path, dT)
    robot = extended_bicycle("RK3", states, controls, path, dT)
        
    control = controllers(robot.gains)    
    
    ekf = Kalman_filter(states.shape[0])
    mc = ManualControl()
    mpc = NonlinearMPC(2.5, dT)

    print("Start Simulation")
    while utils.euclidean_distance(goal, robot.states) > 0.3:
                
#        robot.controls = mc.get_controls(anime.fig, robot.states, dT)
        control.lqr_vel_steer_control(robot)
        robot.controls = mpc.MPC(robot.states, robot.path)
        
        robot_ground_truth.controls = copy.deepcopy(robot.controls)
#        
        add_noise = False
        robot_ground_truth.run(add_noise) # ground truth
#        add_noise = True
        robot.run(add_noise)
#        robot = ekf.run_filter(robot, robot_ground_truth.states) 
#        utils.collision_square_obstacle(robot.states, )
        states_history = np.hstack((states_history, robot.states))
        states_history_ground_truth = np.hstack((states_history_ground_truth, robot_ground_truth.states))
        controls_history = np.hstack((controls_history, robot.controls))
        time += dT
#        plt.pause(0.25)
        if play_animation:
            anime.animate(states_history, robot_ground_truth, path , maps)         
            
    print("Simulation Finished")

