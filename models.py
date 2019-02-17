"""
Robotic Models Class
Nonlinear models and jacobian for:
    - Simple Car/Bicycle
    - Differential Drive
    - Extended Bicycle Kinematics
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

class models():
    
    def __init__(self, model, x, dT=1.0):
        """
        Initialize a robot model
        """
        self.dT = dT
        self.system = x
        if model == "simple bicycle":
            self.model = self.simple_bicycle
        elif model == "differential drive" :
            self.model = self.diff_drive
        elif model == "bicycle kinematic":
            self.model = self.bicycle_kinematic
    
    def simple_bicycle(self, states, u):
        """
        Simple bicycle/car nonlinear system.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        L = 4.5
        max_vel = 10.0
        min_vel = -10.0

        v = u[0]
        df = math.atan(u[1]*L/v)
        #df = u[1]
        if v < min_vel:
            v = min_vel
        elif v > max_vel:
            v = max_vel
        max_df = math.pi/3
        min_df = -math.pi/3
        if df < min_df:
            df = min_df
        elif df > max_df:
            df = max_df
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [truncate_angle(v*np.tan(df)/L)]])
#        system = np.array([[v*np.cos(theta)],
#                   [v*np.sin(theta)],
#                   [df]])
    
        return system
        
    def extended_bicycle(self, states, u, params):
        """
        Simple bicycle/car nonlinear system.
        Input: x - state vector (x, y, theta, velocity, steering angle)
               u - controls (acceleration, steering velocity)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        L = 0.1
        max_vel = 2.0
        min_vel = -2.0
        a = u[0]
        w = u[1]#%2*math.pi
        
        if a < min_vel:
            a = min_vel
        elif a > max_vel:
            a = max_vel
            
            
        if w < min_vel:
            w = min_vel
        elif w > max_vel:
            w = max_vel
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        v = states[3][0]
        phi = states[4][0]
        
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [v*np.tan(phi)/L],
                           [a],
                           [w]])
    
        return self.system   
 
    def front_wheel_bicycle(self, states, u, params):
        """
        Simple front wheel bicycle/car kinematics.
        Input: x - state vector (x, y, theta, velocity, steering angle)
               u - controls (acceleration, steering velocity)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        L = 0.1
        max_vel = 2.0
        min_vel = -2.0
        v = u[0]
        w = u[1]#%2*math.pi
        
        if v < min_vel:
            v = min_vel
        elif v > max_vel:
            v = max_vel
            
            
        if w < min_vel:
            w = min_vel
        elif w > max_vel:
            w = max_vel
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        phi = states[3][0]
        
        system = np.array([[v*np.cos(theta)*np.cos(phi)],
                           [v*np.sin(theta)*np.cos(phi)],
                           [v*np.sin(phi)/L],
                           [w]])
    
        return self.system   
       


    
    def bicycle_kinematic(self, states, u):
        """
        Bicycle Kinematic model

        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """

        lr = 1.738
        lf = 1.105
        L = lr + lf
        a = u[0]
        a = 0.5
        if a < 0.1:
            a = 0.1
        elif a > 1.0:
            a = 1.0
 
        df = u[1]
        df = 0.5
        if df < -math.pi/4:
            df = -math.pi/4
        elif df > math.pi/4:
            df = math.pi/4 
            
        x = states[0][0]
        y = states[1][0]
        phi = states[2][0]
        v = states[3][0]
        beta = states[4][0]
        pb = phi + beta
        
        system = np.array([[v*np.cos(pb)],
                           [v*np.sin(pb)],
                           [(v/lr)*np.sin(beta)],
                           [a],
                           [np.arctan((lr/L)*np.tan(df))]])

        return system
    
    def diff_drive(self, states, u):
        """
        Differential Drive model.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (wheel radius)
        Output: system - system states
        """
        r = 0.05
        L = 0.1
        
        x = states[0][0]
        y = states[1][0]
        phi = states[2][0]
        

        v = u[0]
        df = u[1]
        system = np.array([[r*v*np.cos(phi)],
                           [r*v*np.sin(phi)],
                           [truncate_angle( r*df/L)]])
    
        return system
    
    def simple_bicycle_jacobian(self, x, u, params):
        """
        Simple bicycle/car linearization/jacobian.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        L = params['length']
        dT = self.dT
        v = u[0][0]
        df = u[1][0]
        self.system = np.array([[1.0, 0.0, -dT*v*np.sin(x[2][0])],
                           [0.0, 1.0, dT*v*np.cos(x[2][0])],
                           [0.0, 0.0, 1.0]])
        
        return self.system
        

 
    def diff_drive_jacobian(self, x, u, params):
        """
        Differential Drive model linearization/jacobian.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (wheel radius, distance from front and back axis)
        Output: system - system states
        """
        r = params['radius']
        v = u[0][0]
        df = u[1][0]
        dT = self.dT
        self.system = np.array([[1.0, 0.0, -dT*r*v*np.sin(x[2][0])],
                           [0.0, 1.0, dT*r*v*np.cos(x[2][0])],
                           [0.0, 0.0, 1.0]])
        
        return self.system    
    
    
    def euler_solver(self, x0, t, h, u):
        """
        Euler method for solving first order degree differential equations
        """
        n = (int)((t)/h)
        y = x0
        for i in range(1, n + 1):
            y = y + h*self.model(y, u)
        return y

    def runge_kutta_solver(self, q0, t, h, u):
        """
        Runge Kutta fourth order numerical solver for ordinary differential equations
        """
        n = (int)((t)/h)  
        q = q0 
        for i in range(1, n + 1): 
            k1 = h * self.model(q, u) 
            k2 = h * self.model(q + 0.5 * k1, u) 
            k3 = h * self.model(q + 0.5 * k2, u) 
            k4 = h * self.model(q + k3, u) 
          
            q = q + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4) 
              
        return q 
    
# helper function to map all angles onto [-pi, pi]
def truncate_angle(angle):
    while angle < 0.0:
        angle += math.pi * 2
    return ((angle + math.pi) % (math.pi * 2)) - math.pi
    
def constrain_angle(angle, max_value, min_value):
    if angle < min_value:
        angle = min_value
    elif angle > max_value:
        angle = max_value
    return angle
      

        