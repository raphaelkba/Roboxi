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
        
            
    def model(self, states, u):
        pass            
            
    def control(self, states, reference):
        pass

    def system_constraints(self, states, controls):
        pass

    def run(self):
        pass

    
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


    def euler_solver(self):
        """
        Euler method for solving first order degree differential equations
        """
        h = self.dT/self.euler_step
        n = (int)((self.dT)/h)
        for i in range(1, n + 1):
            self.states = self.states + h*self.model(self.states, self.controls)
        
    
    def runge_kutta_solver(self):
        """
        Runge Kutta fourth order numerical solver for ordinary differential equations
        """
        h = self.dT/self.rk4_step
        n = (int)((self.dT)/h)  
        for i in range(1, n + 1): 
            k1 = h * self.model(self.states , self.controls) 
            k2 = h * self.model(self.states  + 0.5 * k1, self.controls) 
            k3 = h * self.model(self.states  + 0.5 * k2, self.controls) 
            k4 = h * self.model(self.states  + k3, self.controls) 
          
            self.states  = self.states  + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4) 

class simple_bicycle(robots):
    
    def __init__(self, robot, states, controls, path, dT=1.0):
        self.L = 4.5
        self.max_vel = 50.0
        self.min_vel = -50.0
        self.max_steering_angle = math.pi/3
        self.min_steering_angle = -math.pi/3
        self.gains = np.array([0.5, 1.0, -0.5])
        self.lookahead_idx = 5
        super().__init__(robot, states, controls, path, dT=1.0)
        
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
        
    def control(self, states, reference):
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
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel) # constrain velocity
        controls[1] = utils.constrain_value(controls[1], self.min_steering_angle, self.max_steering_angle) # constrain steering angle
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.goal = self.look_ahead(self.lookahead_idx)
        self.controls = self.control(self.states, self.goal)
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        

class diff_drive(robots):


    def __init__(self, robot, states, controls, path, dT=1.0):
        self.r = 0.05
        self.L = 0.1
        self.max_vel = 2.0
        self.min_vel = -2.0
        self.gains = np.array([2.5, 1.0, -0.5])
        self.lookahead_idx = 5
        super().__init__(robot, states, controls, path, dT=1.0)
        
    def model(self, states, u):
        """
        Differential Drive model.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (wheel radius)
        Output: system - system states
        """
        
        v = u[0]
        df = u[1][0]
        
        # set contraints
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
                
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]

        system = np.array([[self.r*v*np.cos(theta)],
                           [self.r*v*np.sin(theta)],
                           [utils.truncate_angle(self.r*df/self.L)]])
       
        return system
    
    def control(self, states, reference):
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
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel) # constrain velocity
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.goal = self.look_ahead(self.lookahead_idx)
        self.controls = self.control(self.states, self.goal)
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        
        
class extended_bicycle(robots):


    def __init__(self, robot, states, controls, path, dT=1.0):
        self.L = 4.5
        self.max_vel = 0.1
        self.min_vel = -0.1
        self.max_acc = 0.1
        self.min_acc = -0.1        
        self.max_steering_vel = 0.1
        self.min_steering_vel = -0.1
        self.max_steering_angle = math.pi/3
        self.min_steering_angle = -math.pi/3
        self.gains = np.array([0.1, 0.0, 1.0, -0.5])
        self.lookahead_idx = 5
        self.error_old = 0
        super().__init__(robot, states, controls, path, dT=1.0)
        
    def model(self, states, u):
        """
        Extended bicycle model.
        Input: x - state vector (x, y, theta, velocity, steering angle)
               u - controls (acceleration, steering velocity)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        a = u[0] # velocity control
        w = u[1][0]
        
        # set contraints
        a = utils.constrain_value(a, self.min_acc, self.max_acc)
        w = utils.constrain_value(w, self.min_steering_vel, self.max_steering_vel)
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        v = states[3][0]
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
        phi = states[4][0]
        phi = math.atan(phi*self.L/v) # transformation of steering angle to robots heading
        if np.isnan(phi):
            phi = 0
        
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [utils.truncate_angle(v*np.tan(phi)/self.L)],
                           [a],
                           [w]])
          
        
        return system   
        
    def control(self, states, reference):
        # distance to goal
        distance = utils.euclidean_distance(states, reference)
               
        
        # PID controller for velocity
        velocity_control = self.gains[0]*distance
        # angle control
        angle_to_point = (utils.angle_between_points(states, reference) - states[2] + math.pi)%(2*math.pi) - math.pi
        
        # derivative term         
        error_diff = angle_to_point - self.error_old
        self.error_old = angle_to_point

        beta = (reference[2] - states[2] - angle_to_point + math.pi) % (2 * math.pi) - math.pi
        # maneuver
        if angle_to_point > math.pi / 2 or angle_to_point < -math.pi / 2:
            velocity_control = -velocity_control

        
        return np.array([velocity_control, self.gains[2]*angle_to_point + self.gains[3]*beta + self.gains[1]*error_diff])        
        
        
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        states[3][0] = utils.constrain_value(states[3][0], self.min_vel, self.max_vel) # orientation from -pi to pi 
        states[4][0] = utils.truncate_angle(states[4][0]) # orientation from -pi to pi 
        
        controls[0] = utils.constrain_value(controls[0], self.min_acc, self.max_acc) # constrain acc
        controls[1][0] = utils.constrain_value(controls[1][0], self.min_steering_vel, self.max_steering_vel) # constrain acc
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.goal = self.look_ahead(self.lookahead_idx)
        self.controls = self.control(self.states, self.goal)
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)