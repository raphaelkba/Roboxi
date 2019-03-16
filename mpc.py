# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 01:10:46 2019

@author: rapha
"""

import numpy as np
from sys import path
path.append(r"C:\Users\rapha\Documents\Projects\casadi-windows-py36-v3.4.5-64bit")
from casadi import *
import copy

N = 15 # number of control intervals
lr = 1.0
T = N


u_1 = np.zeros((N))
u_2 = np.zeros((N))
x_init = 0.0
y_init = 0.0
theta_init = 0.0
    
for t in range(N):
    # ---- objective          ---------
    opti = Opti() # Optimization problem
    
    # ---- decision variables ---------
    X = opti.variable(3,N) # state trajectory
    x 	   = X[0,:]
    y 	   = X[1,:]	       
    #v     = X[2,:]          
    theta = X[2,:]      
    

    
    U = opti.variable(2,N)   # control trajectory (throttle)
    a = U[0,:]
    steer_angle = U[1,:]

    #    V = (x[i]-pathx[i])**2+(y[i]-pathy[i])**2 + a[i]*a[i] + steer_angle[i]*steer_angle[i]
    V = 0
    #pathy = [0.0, 2.0, 3.0, 5.0, 5.0]
    #pathx = [0.0, 1.0, 3.0, 3.0, 3.0]
    
    for i in range(N):
        V += (x[i]-5.0)**2+(y[i]-12.0)**2 #+ a[i]*a[i] + steer_angle[i]*steer_angle[i]
    opti.minimize(V) # race in minimal time
    
    f = lambda x,u: vertcat(u[0]*casadi.cos(x[2] + u[1]), u[0]*casadi.sin(x[2] + u[1]), u[0]* casadi.sin(u[1])/lr )
    

    dt = T/N # length of a control interval
    for k in range(N-1): # loop over control intervals
       # Runge-Kutta 4 integration
       k1 = f(X[:,k],         U[:,k])
       k2 = f(X[:,k]+dt/2*k1, U[:,k])
       k3 = f(X[:,k]+dt/2*k2, U[:,k])
       k4 = f(X[:,k]+dt*k3,   U[:,k])
       x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
       opti.subject_to(X[:,k+1]==x_next) # close the gaps
    
    # ---- path constraints -----------
    #limit = lambda pos: 1-sin(2*pi*pos)/2
    #opti.subject_to(speed<=limit(pos))   # track speed limit
    #opti.subject_to(opti.bounded(0,U,1)) # control is limited
    
    # ---- boundary conditions --------
    #for i in range(len(pathx)):
    #    opti.subject_to(x[i]==pathx[i])   # start at position 0 ...
    #    opti.subject_to(y[i]==pathy[i])   
    opti.subject_to(x[0]==x_init)
    opti.subject_to(y[0]==y_init)
    opti.subject_to(theta[0]==theta_init)

    opti.subject_to(x[-1]==5)
    opti.subject_to(y[-1]==12)
    #opti.subject_to(v[0]==0) # ... from stand-still 

    opti.subject_to(opti.bounded(-1,a,1))  # finish line at position 1
    opti.subject_to(opti.bounded(-1,steer_angle,1)) 
    ## ---- misc. constraints  ----------
    #opti.subject_to(T>=0) # Time must be positive
    
    # ---- initial values for solver ---
    for n in range(N):
        opti.set_initial(U[0,i], u_1[i])
        opti.set_initial(U[1,i], u_2[i])
 
#    opti.set_initial(U[0,0], u_1[0])
#    opti.set_initial(U[0,1], u_1[1])
#    opti.set_initial(U[0,2], u_1[2])
#    opti.set_initial(U[0,3], u_1[2])
#    opti.set_initial(U[0,4], u_1[3])
#    opti.set_initial(U[0,5], u_1[4])
#    opti.set_initial(U[1,0], u_2[0])
#    opti.set_initial(U[1,1], u_2[1])
#    opti.set_initial(U[1,2], u_2[2])
#    opti.set_initial(U[1,3], u_2[3])
#    opti.set_initial(U[1,4], u_2[4])
#    opti.set_initial(U[1,5], u_2[5])
#    #opti.set_initial(T, 1)
    
    # ---- solve NLP              ------
    p_opts = {"expand":True}
    s_opts = {"max_iter": 1000000000}
    opti.solver("ipopt",p_opts,
                    s_opts)
    sol = opti.solve()   # actual solve
    x_init = copy.deepcopy(sol.value(x[1]))
    y_init = copy.deepcopy(sol.value(y[1]))
    theta_init = copy.deepcopy(sol.value(theta[1]))
    
    print("acc: ",  sol.value(U[0,0]))
    print("steering: ",  sol.value(U[1,0]))
    
    for i in range(N-1):
        u_1[i] = copy.deepcopy(sol.value(U[0,i+1]))
        u_2[i] = copy.deepcopy(sol.value(U[1,i+1]))
    u_1[-1] = 0
    u_2[-1] = 0
    # ---- post-processing        ------
    from pylab import plot, step, figure, legend, show, spy
    figure(1)
#    #plot(sol.value(a),label="speed")
    plot(sol.value(x[0:2]),sol.value(y[0:2]),label="speed")
#    figure()
    a = plot(sol.value(x),sol.value(y),".")
    #plot(sol.value(x),label="pos")
    #plot(limit(sol.value(x)),'r--',label="speed limit")
#    figure(2)
#    step(range(N+1),sol.value(U[0,:]),'k',label="throttle")
#    figure(3)
#    step(range(N+1),sol.value(U[1,:]),'r',label="throttle")
    #legend(loc="upper left")
    
    #figure()
    #spy(sol.value(jacobian(opti.g,opti.x)))
    #figure()
    #spy(sol.value(hessian(opti.f+dot(opti.lam_g,opti.g),opti.x)[0]))
    del(opti)
    show()
    plt.pause(0.1)
    