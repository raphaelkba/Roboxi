"""
Robots Class
Nonlinear models and jacobian for:
    - Simple Car/Bicycle
    - Differential Drive
    - Extended Bicycle Kinematics
    - Front Wheel Drive
Solvers: 
        - Euler solver
        - Runge Kutta 4
Sources: http://planning.cs.uiuc.edu/node657.html
         https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""


import numpy as np
import math
from utils import utils

class simple_bicycle():
    
    def __init__(self):
        self.L = 4.5
        self.max_vel = 50.0
        self.min_vel = -50.0
        self.max_steering_angle = math.pi/3
        self.min_steering_angle = -math.pi/3
        self.gains = np.array([1.0, 1.0, -0.5])
        self.lookahead_idx = 2
        
    def model(self, states, u):
        """
        Simple bicycle/car nonlinear system.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
        Output: system - system states
        """
 
        v = u[0] # velocity control
        df = math.atan(u[1]*self.L/v) # transformation of steering angle to robots heading
        
        # set contraints
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
        df = utils.constrain_value(df, self.min_steering_angle, self.max_steering_angle)
        # states
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        # update
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [utils.truncate_angle(v*np.tan(df)/self.L)]])

        return system
        
    def simple_pose_control(self, states, reference):
       # distance to goal
       distance = utils.euclidean_distance(states, reference)
    
       # Velocity controller
       velocity_control = self.gains[0]*distance
       # angle control
       angle_to_point = (utils.angle_between_points(states, reference) - states[2] + math.pi)%(2*math.pi) - math.pi
       beta = (reference[2] - states[2] - angle_to_point + math.pi) % (2 * math.pi) - math.pi
       steering_control = self.gains[1]*angle_to_point + self.gains[2]*beta
       # maneuver
       if angle_to_point > math.pi / 2 or angle_to_point < -math.pi / 2:
           velocity_control = -velocity_control
           
       return np.array([velocity_control, steering_control])
        
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0])
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel)
        controls[1] = utils.constrain_value(controls[1], self.min_steering_angle, self.max_steering_angle)
        return states, controls

class robots():     
    
    def __init__(self, robot, states, controls, path, dT=1.0):
        """
        Initialize a robot model
        """
        self.dT = dT
        self.states = states
        self.controls = controls
        self.rk4_step = 10
        self.euler_step = 10
        self.path = path
        self.goal = []
        
        if robot == "simple bicycle":
            self.robot = simple_bicycle()
            

    def run(self):
        self.goal = self.look_ahead(self.robot.lookahead_idx)
        self.controls = self.robot.simple_pose_control(self.states, self.goal)
        self.runge_kutta_solver()
        self.states, self.controls = self.robot.system_constraints(self.states, self.controls)

            
    def euler_solver(self):
        """
        Euler method for solving first order degree differential equations
        """
        h = self.dT/self.euler_step
        n = (int)((self.dT)/h)
        for i in range(1, n + 1):
            self.states = self.states + h*self.robot.model(self.states, self.controls)
        
    
    def runge_kutta_solver(self):
        """
        Runge Kutta fourth order numerical solver for ordinary differential equations
        """
        h = self.dT/self.rk4_step
        n = (int)((self.dT)/h)  
        for i in range(1, n + 1): 
            k1 = h * self.robot.model(self.states , self.controls) 
            k2 = h * self.robot.model(self.states  + 0.5 * k1, self.controls) 
            k3 = h * self.robot.model(self.states  + 0.5 * k2, self.controls) 
            k4 = h * self.robot.model(self.states  + k3, self.controls) 
          
            self.states  = self.states  + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4) 
    
    def look_ahead(self, lookahead_idx):    
        """
        search for the closest point on the reference path to the robot and return 
        a look ahed point (a point a little further than the closest one)
        """
        min_dist = float('inf')
        dist = 0.0
        idx = 0
        goal = [0.0, 0.0, 0.0]
        for i in range(len(self.path[0])-lookahead_idx): # find closest path point 
            dist = utils.euclidean_distance((self.path[0][i],self.path[1][i]), self.states)
            if dist < min_dist:
                min_dist = dist
                idx = i
                goal[0] = self.path[0][i+lookahead_idx]
                goal[1] = self.path[1][i+lookahead_idx]
                goal[2] = math.atan2(self.path[1][i+1]-self.path[1][i], self.path[0][i+1]- self.path[0][i])
                if len(self.path[0]) < lookahead_idx+1: # if close to final goal, returns final goal 
                    goal[0] = self.path[0][-1]
                    goal[1] = self.path[1][-1]
                    goal[2] = math.atan2(self.path[1][i+1]-self.path[1][i], self.path[0][i+1]- self.path[0][i])
        for i in range(idx): # delete passed path points
            self.path = np.delete(self.path,[i],1)
        return goal
