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
    
    def __init__(self, robot, states, controls, path, dT):
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
        self.robot_name = robot
        
            
    def model(self, states, u):
        pass            
            
    def control(self, states, reference):
        pass

    def system_constraints(self, states, controls):
        pass

    def run(self):
        pass

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
    
    def __init__(self, robot, states, controls, path, dT):
        self.L = 4.5
        self.max_vel = 50.0
        self.min_vel = -50.0
        self.max_steering_angle = math.pi/3
        self.min_steering_angle = -math.pi/3
        self.gains = np.array([0.5, 0.0, 1.0, -0.5])
        self.lookahead_idx = 5
        super().__init__(robot, states, controls, path, dT)
        
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
        
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel) # constrain velocity
        controls[1] = utils.constrain_value(controls[1], self.min_steering_angle, self.max_steering_angle) # constrain steering angle
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        

class diff_drive(robots):


    def __init__(self, robot, states, controls, path, dT):
        self.r = 0.1
        self.L = 0.5
        self.max_vel = 2.0
        self.min_vel = -2.0
        self.gains = np.array([2.5, 0.0, 1.0, -0.5])
        self.lookahead_idx = 15
        super().__init__(robot, states, controls, path, dT)
        
    def model(self, states, u):
        """
        Differential Drive model.
        Input: x - state vector (x, y, theta)
               u - controls (speed, steering_angle)
               params - parameters (wheel radius)
        Output: system - system states
        """
        
        v = u[0]
        w = u[1]
        
        # set contraints
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
                
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        phi = states[3][0]

        system = np.array([[self.r*v*np.cos(theta)],
                           [self.r*v*np.sin(theta)],
                           [utils.truncate_angle(self.r*phi/self.L)],
                           [w]])
       
        return system
       
    def jacobi(self, vel):

        theta = self.states[2][0]
        
        A = np.zeros((3, 3))
        A[0, 0] = 1.0
        A[0, 1] = 0.0
        A[0, 2] = -self.dT*self.r*vel*np.sin(theta)
        A[1, 0] = 0.0
        A[1, 1] = 1.0
        A[1, 2] = self.dT*self.r*vel*np.cos(theta)
        A[2, 0] = 0.0
        A[2, 1] = 0.0
        A[2, 2] = 1.0

        
        B = np.zeros((3, 2))
        B[0, 0] = self.dT*self.r*np.cos(theta)
        B[1, 0] = self.dT*self.r*np.sin(theta)
        B[2, 1] = (self.dT*self.r)/self.L
      
      
        return A, B
        
    def error(self):
        angle_to_point = (utils.angle_between_points(self.states, self.goal) - self.states[2][0] + math.pi)%(2*math.pi) - math.pi
        error = [[0],[0],[0]]
        error[0] = self.states[0][0] - self.goal[0]
        error[1] = self.states[1][0] - self.goal[1]
        error[2] = -angle_to_point

        return error        
        
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel) # constrain velocity
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.controls[1] = (self.controls[1] - self.states[3][0])/self.dT
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        
        
class extended_bicycle(robots):


    def __init__(self, robot, states, controls, path, dT):
        self.L = 4.5
        self.max_vel = 50.0
        self.min_vel = -50.0
        self.max_acc = 5.0
        self.min_acc = -5.0        
        self.max_steering_vel = 1.0
        self.min_steering_vel = -1.0
        self.max_steering_angle = math.pi/3
        self.min_steering_angle = -math.pi/3
        self.gains = np.array([0.5, 5.0, 1.0, -0.5])
        self.lookahead_idx = 10
        self.error_old = 0
        self.previous_error = 0
        self.previous_angle_error = 0
        self.Q = np.eye(5)
        self.R = np.eye(2)
        super().__init__(robot, states, controls, path, dT)
        
    def model(self, states, u):
        """
        Extended bicycle model.
        Input: x - state vector (x, y, theta, velocity, steering angle)
               u - controls (acceleration, steering velocity)
               params - parameters (distance from front and back axis)
        Output: system - system states
        """
        a = u[0] # velocity control
        w = u[1]
        
        # set contraints
        a = utils.constrain_value(a, self.min_acc, self.max_acc)
        w = utils.constrain_value(w, self.min_steering_vel, self.max_steering_vel)
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        v = states[3][0]
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
        phi = states[4][0]

        
        system = np.array([[v*np.cos(theta)],
                           [v*np.sin(theta)],
                           [utils.truncate_angle(v*np.tan(phi)/self.L)],
                           [a],
                           [w]])
          
        
        return system   
       
       
    def jacobi(self, vel):
        theta = self.states[2][0]
        v = self.states[3][0]        
        phi = self.states[4][0]
        
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = 0.0
        A[0, 2] = -self.dT*v*np.sin(theta)
        A[0, 3] = self.dT*np.cos(theta)
        A[0, 4] = 0
        A[1, 0] = 0.0
        A[1, 1] = 1.0
        A[1, 2] = self.dT*v*np.cos(theta)
        A[1, 3] = self.dT*np.sin(theta)
        A[1, 4] = 0.0
        A[2, 0] = 0.0
        A[2, 1] = 0.0
        A[2, 2] = 1.0
        A[2, 3] = self.dT*np.tan(phi)/self.L
        A[2, 4] = self.dT*v/(np.cos(phi)*np.cos(phi)*self.L)
        A[3, 0] = 0.0        
        A[3, 1] = 0.0    
        A[3, 2] = 0.0
        A[3, 3] = 1.0        
        A[3, 4] = 0.0
        A[4, 0] = 0.0       
        A[4, 1] = 0.0        
        A[4, 2] = 0.0        
        A[4, 3] = 0.0        
        A[4, 4] = 1.0   
        
        B = np.zeros((5, 2))
        B[3, 0] = self.dT
        B[4, 1] = self.dT   
        
        return A, B
        
    def error(self):
        angle_to_point = (utils.angle_between_points(self.states, self.goal) - self.states[2][0] + math.pi)%(2.0*math.pi) - math.pi
        beta = (self.goal[2] - self.states[2][0] - angle_to_point + math.pi) % (2.0 * math.pi) - math.pi
        # derivative term for angle control         
        error_diff = angle_to_point - self.error_old
        self.error_old = angle_to_point        
        distance =- utils.euclidean_distance(self.states, self.goal)
        
        error = [[0],[0],[0],[0],[0]]
        error[0] = self.states[0][0] - self.goal[0]
        error[1] = self.states[1][0] - self.goal[1]
        error[2] = - angle_to_point + 0.1*beta
        error[3] = self.states[3][0] - 0.0
        error[4] = - angle_to_point + 0.1*beta
        
        self.previous_error = distance
        self.previous_angle_error = angle_to_point
        return error
       
     
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        states[3][0] = utils.constrain_value(states[3][0], self.min_vel, self.max_vel) # orientation from -pi to pi 
        states[4][0] = utils.constrain_value(states[4][0], self.min_steering_angle, self.max_steering_angle) # orientation from -pi to pi 
        
        controls[0] = utils.constrain_value(controls[0], self.min_acc, self.max_acc) # constrain acc
        controls[1] = utils.constrain_value(controls[1], self.min_steering_vel, self.max_steering_vel) # constrain acc
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.controls[0] = (self.controls[0] - self.states[3][0])/self.dT
        self.controls[1] = (self.controls[1] - self.states[4][0])/self.dT
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        
        
class front_wheel_drive(robots):
    
    def __init__(self, robot, states, controls, path, dT):
        self.L = 4.5
        self.min_vel = -50.0
        self.max_vel = 50.0
        self.min_steering_vel = -1.0
        self.max_steering_vel = 1.0
        self.gains = np.array([0.3, 0.0, 1.0, -0.5])
        self.lookahead_idx = 5
        super().__init__(robot, states, controls, path, dT)
        
        
    def model(self, states, u):
        """
        Front wheel drive.
        Input: x - state vector (x, y, theta, phi)
               u - controls (speed, steering_angle)
        Output: system - system states
        """
        v = u[0]
        w = u[1]
        
        # set contraints
        v = utils.constrain_value(v, self.min_vel, self.max_vel)
        w = utils.constrain_value(w, self.min_steering_vel, self.max_steering_vel)        
            
        x = states[0][0]
        y = states[1][0]
        theta = states[2][0]
        phi = states[3][0]
        
        system = np.array([[v*np.cos(theta)*np.cos(phi)],
                           [v*np.sin(theta)*np.cos(phi)],
                           [v*np.sin(phi)/self.L],
                           [w]])
    
        return system   

        
    def system_constraints(self, states, controls):
        states[2][0] = utils.truncate_angle(states[2][0]) # orientation from -pi to pi 
        controls[0] = utils.constrain_value(controls[0], self.min_vel, self.max_vel) # constrain velocity
        controls[1] = utils.constrain_value(controls[1], self.min_steering_vel, self.max_steering_vel) # constrain steering angle
        return states, controls
        
    def run(self):
        """ simulation pipeline for running the robot one timestep """
        self.controls[1] = (self.controls[1] - self.states[3][0])/self.dT
        self.runge_kutta_solver()
        self.states, self.controls = self.system_constraints(self.states, self.controls)
        
        
        