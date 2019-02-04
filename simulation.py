"""
Robotic Simulation 

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from models import models

if __name__ == '__main__':
    simulate = True
    animation = True
    dT = 1.0
    params = {}
    params['length'] = 1.0
    params['radius'] = 0.5
    
    
    states = np.array([[0.0],
                      [0.0],
                      [0.0]])
    
    r = np.array([[0.0],
                  [0.0],
                  [0.0]])
    
    controls = np.array([[1.1],
                        [0.1]])
    
    
    robot = models(states, dT)
    
    time = 0.0
    states_history = states
    states_history2 = states
    print("Start Simulation")

    while simulate:
        
        robot.simple_bicycle(controls, params)
        r = robot.euler_solver(r, dT, dT/1, controls)
        print("euler")
        print(robot.system)
        print("RK4")
        print(r)
        
        
        states_history = np.hstack((states_history, robot.system))
        states_history2 = np.hstack((states_history2, r))
        time += dT
        
        if animation:
            plt.cla()
            plt.plot(states_history[0][:],states_history[1][:], ".r")
            plt.plot(states_history2[0][:],states_history2[1][:], ".b")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.1)