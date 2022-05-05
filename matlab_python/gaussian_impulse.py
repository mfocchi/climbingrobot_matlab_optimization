# -*- coding: utf-8 -*-
"""
Created on Wed May  4 11:04:16 2022

@author: mfocchi
"""

import numpy as np
import matplotlib.pyplot as plt
import math

def gaussian(x,a,mu,sigma):
    return a*math.exp(-(x-mu)**2/(2*sigma**2))

    
m = 5 #kg
N = 500
T_f = 2.
T_thrust = 0.2
time = np.linspace(0, T_f, N)
max_force = m*9.81 * 3 

# gaussian parameters
sigma = 1/3. *(T_thrust/2.)
mean = T_thrust/2 


force = []

for i in range(len(time)):
    force.append( gaussian(time[i], max_force, mean, sigma) )


fig = plt.figure()
plt.plot(time, force, "ob")
plt.xlim(0, T_thrust + 0.1)
plt.ylim(0, max_force)
plt.title("gaussian impulse")
plt.xlabel("time [$s$]")
plt.ylabel("force [$N$]")
plt.grid()
plt.legend()
plt.savefig('../figs/gaussian.png')
plt.show()