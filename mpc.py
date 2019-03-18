# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 01:10:46 2019
Nonlinear Model Predictive Control
#     CasADi -- A symbolic framework for dynamic optimization.
#     Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
#                             K.U. Leuven. All rights reserved.
#     Copyright (C) 2011-2014 Greg Horn
#     CasADi is free software; you can redistribute it and/or
#     modify it under the terms of the GNU Lesser General Public
#     License as published by the Free Software Foundation; either
#     version 3 of the License, or (at your option) any later version.
#
#     CasADi is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#
#     You should have received a copy of the GNU Lesser General Public
#     License along with CasADi; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
source: https://web.casadi.org/
@author: rapha
"""

import numpy as np
from sys import path
path.append(r"C:\Users\rapha\Documents\Projects\casadi-windows-py36-v3.4.5-64bit")
from casadi import *
import copy
from utils import utils
import math
import matplotlib.pyplot as plt

class NonlinearMPC():
    
    def __init__(self, N, dT):
        self.N = N # prediction horizon in seconds
        self.dT = dT # timestep
        self.H = int(N/dT) # prrdiction horizon steps
        self.u_1 = np.zeros((self.H+1)) # acceleration control
        self.u_2 = np.zeros((self.H+1)) # steering velocity control
        self.lr = 4.5 # car data
 
    def MPC(self, states, path):
        
        opti = Opti() # Optimization problem
        
        # system states and controls
        X = opti.variable(5, self.H+1) # state trajectory   
        x = X[0,:]
        y = X[1,:]	               
        theta = X[2,:]  
        v = X[3,:]
        phi = X[4,:]
        
        U = opti.variable(2,self.H+1)   # control trajectory (acceleration and steering velocity)
        a = U[0,:]
        steer_angle = U[1,:]
        
        # cost function
        V = 0    
        for i in range(self.H+1):
            if i < len(path[0]):
                V += (x[i]-path[0][i])**2+(y[i]-path[1][i])**2 #+ a[i]*a[i] + steer_angle[i]*steer_angle[i]
            else:
                V += (x[i]-path[0][-1])**2+(y[i]-path[1][-1])**2 # + a[i]*a[i] + steer_angle[i]*steer_angle[i]
    #    V += (casadi.fabs(casadi.cos(theta[-1]) - casadi.cos(pi))**2 + casadi.fabs(casadi.sin(theta[-1]) - casadi.sin(pi))**2)
        opti.minimize(V)
        
        # system
        f = lambda x,u: vertcat(x[3,:]*casadi.cos(x[2,:]), 
                                x[3,:]*casadi.sin(x[2,:]), 
                                x[3,:]*casadi.tan(x[4,:])/self.lr,
                                u[0,:],
                                u[1,:])
        
        # system constraints
        opti.bounded(-math.pi, X[2,:], math.pi)
        opti.bounded(-50, X[3,:], 50)
        opti.bounded(-math.pi/3, X[4,:], math.pi/3)
        opti.subject_to(opti.bounded(-5.0, a, 5.0))  # finish line at position 1
        opti.subject_to(opti.bounded(-math.radians(50.0), steer_angle, math.radians(50.0))) 

        for k in range(self.H): # loop over control intervals
           # Runge-Kutta 4 integration
           k1 = f(X[:,k],         U[:,k])
           k2 = f(X[:,k]+self.dT/2*k1, U[:,k])
           k3 = f(X[:,k]+self.dT/2*k2, U[:,k])
           k4 = f(X[:,k]+self.dT*k3,   U[:,k])
           x_next = X[:,k] + self.dT/6*(k1+2*k2+2*k3+k4) 
           opti.subject_to(X[:,k+1]==x_next) # close the gaps
        
        # initial conditions
        opti.subject_to(x[0]==states[0,0])
        opti.subject_to(y[0]==states[1,0])
        opti.subject_to(theta[0]==states[2,0])
        opti.subject_to(v[0]==states[3,0])
        opti.subject_to(phi[0]==states[4,0])
        
        # initial control conditions
        for n in range(self.H+1):
            opti.set_initial(U[0,n], self.u_1[n])
            opti.set_initial(U[1,n], self.u_2[n])
     
        # solve NLP
        p_opts = {"expand":True}
        s_opts = {"max_iter": 1000}
        opti.solver("ipopt", p_opts, s_opts)
        try:
            sol = opti.solve()
            print("acc: ",  sol.value(U[0,0]))
            print("steering: ",  sol.value(U[1,0]))
            control = np.array([sol.value(U[0,0]), sol.value(U[1,0])])
            
            # shift controls
            for i in range(self.H):
                self.u_1[i] = copy.deepcopy(sol.value(U[0,i+1]))
                self.u_2[i] = copy.deepcopy(sol.value(U[1,i+1]))
            self.u_1[-1] = 0
            self.u_2[-1] = 0

            # ploting
            from pylab import plot, step, figure, legend, show, spy
            figure(1)
            plot(sol.value(x[0:2]),sol.value(y[0:2]),label="speed")
            plot(sol.value(x),sol.value(y),".")

            #figure()
            #spy(sol.value(jacobian(opti.g,opti.x)))
            #figure()
            #spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))
            del(opti)
            show()
            plt.pause(0.05)
            return control
        except:
#            opti.debug.value(X)
#            opti.debug.value(X,opti.initial())
#            opti.debug.show_infeasibilities()
#            opti.debug.x_describe(0)
#            opti.debug.g_describe(0)
#            opti.callback(lambda i: plt.plot(opti.debug.value(X)))
#            plt.show()
#            plt.pause(0.1)
            print("NMPC failed")
            
        # in case it fails use previous computed controls and shift it
        control = np.array([self.u_1[0], self.u_2[0]])
        for i in range(self.H):
            self.u_1[i] = self.u_1[i+1]
            self.u_2[i] = self.u_2[i+1]
        self.u_1[-1] = 0
        self.u_2[-1] = 0
        return control
