"""
Robots Class
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

class simple_bicycle():
    
    def __init__(self, model, x, dT=1.0):
        