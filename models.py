"""
Robotic Models Class
Nonlinera models and jacobian for:
    - Simple Car/Bicycle
    - Differential Drive
Sources: http://planning.cs.uiuc.edu/node657.html

Author: Raphael Kusumoto Barbosa de Almeida
E-Mail : raphael_kba@hotmail.com
"""

import numpy as np
import math


class models():
    
    def __init__(self, x, dT=1.0):
        """
        Initialize a robot model
        """
        self.dT = dT
        self.system = x
    
    
    def simple_bicycle(self, u, params):
        """
        Simple bicycle/car nonlinear system.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
#        L = params['length']
        L = 1
        dT = self.dT
        v = u[0][0]
        df = u[1][0]
        self.system = np.array([[self.system[0][0] + dT*v*np.cos(self.system[2][0])],
                           [self.system[1][0] + dT*v*np.sin(self.system[2][0])],
                           [self.system[2][0] + dT*v*np.tan(df)/L]])
    
        return self.system
    
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
        
    def diff_drive(self, u, params):
        """
        Differential Drive model.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (wheel radius)
        Output: system - system states
        """
        r = params['radius']
        L = params['length']
        dT = self.dT
        v = u[0][0]
        df = u[1][0]
        self.system = np.array([[self.system[0][0] + dT*r*v*np.cos(self.system[2][0])],
                           [self.system[1][0] + dT*r*v*np.sin(self.system[2][0])],
                           [truncate_angle(self.system[2][0] + r*dT*df/L)]])
    
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
    
    def bicycle_kinematic(self, u, params):
        """
        Kinematic Bicycle model
        Source:https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        L = params['length']
        lr = 1.738
        lf = 1.105
        dT = self.dT
        a = u[0][0]
        df = u[1][0]
        x = self.system[0][0]
        y = self.system[1][0]
        phi = self.system[2][0]
        v = self.system[3][0]
        beta = self.system[4][0]
        self.system = np.array([[x + dT*v*np.cos(phi)],
                           [y + dT*v*np.sin(phi)],
                           [phi + dT*v*np.sin(beta)/lf],
                           [v + a],
                           [beta + np.arctan((lr/(lf + lr))*np.tan(df))]])
    
        return self.system
    
    def simple_bicycle_dgl(self, states, u):
        L = 1.0
        v = u[0][0]
        df = u[1][0]
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [v*np.tan(df)/L]])
    
        return system
    
    def euler_solver(self, x0, t, h, u):
        n = (int)((t)/h)
        y = x0
        for i in range(1, n + 1):
            y = y + h*self.simple_bicycle_dgl(y, u)
        return y

    def runge_kutta_solver(self, x0, t, h, u): 
        n = (int)((t)/h)  
        y = x0 
        for i in range(1, n + 1): 
            "Apply Runge Kutta Formulas to find next value of y"
            k1 = h * self.simple_bicycle_dgl(y, u) 
            k2 = h * self.simple_bicycle_dgl(y + 0.5 * k1, u) 
            k3 = h * self.simple_bicycle_dgl(y + 0.5 * k2, u) 
            k4 = h * self.simple_bicycle_dgl(y + k3, u) 
          
            # Update next value of y 
            y = y + (1.0 / 6.0)*(k1 + 2 * k2 + 2 * k3 + k4) 
              
        return y 
    
# helper function to map all angles onto [-pi, pi]
def truncate_angle(angle):
    while angle < 0.0:
        angle += math.pi * 2
    return ((angle + math.pi) % (math.pi * 2)) - math.pi
        
        
        