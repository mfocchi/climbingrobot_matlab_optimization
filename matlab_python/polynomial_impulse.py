# -*- coding: utf-8 -*-
"""
Created on Wed May  4 11:04:16 2022

@author: mfocchi
"""

import numpy as np
import matplotlib.pyplot as plt

def fifthOrderPolynomialTrajectory(tf, start, end):

    # Matrix used to solve the linear system of equations for the polynomial trajectory
    polyMatrix = np.array([[1,  0,              0,               0,                  0,                0],
                           [0,  1,              0,               0,                  0,                0],
                           [0,  0,              2,               0,                  0,                0],
                           [1, tf,np.power(tf, 2), np.power(tf, 3),    np.power(tf, 4),  np.power(tf, 5)],
                           [0,  1,           2*tf,3*np.power(tf,2),   4*np.power(tf,3), 5*np.power(tf,4)],
                           [0,  0,              2,             6*tf, 12*np.power(tf,2),20*np.power(tf,3)]])
    
    polyVector = np.array([start, 0, 0, end, 0, 0])
    matrix_inv = np.linalg.inv(polyMatrix)
    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff
    
    
m = 5 #kg
N = 500
T_f = 2.
T_thrust = 0.2
time = np.linspace(0, T_f, N)

dt = T_f/N
max_force = m*9.81 * 3 

# polynomials coefficients
a_up = fifthOrderPolynomialTrajectory(T_thrust/2, 0, max_force)
a_down = fifthOrderPolynomialTrajectory(T_thrust/2, max_force, 0)
    
force = []

# first half
t = 0. 
for i in range( int(T_thrust/(2*dt)) ):
    force.append( a_up[0] + a_up[1]*t  + a_up[2]*t**2 + a_up[3]*t**3 + a_up[4]*t**4 + a_up[5]*t**5)
    t += dt
# second half
t = 0. 
for i in range( int(T_thrust/(2*dt)) ):
    force.append( a_down[0] + a_down[1]*t  + a_down[2]*t**2 + a_down[3]*t**3 + a_down[4]*t**4 + a_down[5]*t**5)
    t += dt    
# append zeros till the end
for i in range( int((T_f - T_thrust)/dt) ):
    force.append( 0.)
 
       
fig = plt.figure()
plt.plot(time, force, "ob")
plt.xlim(0, T_thrust + 0.1)
plt.ylim(0, max_force)
plt.title("polynomial impulse")
plt.xlabel("time [$s$]")
plt.ylabel("force [$N$]")
plt.grid()
plt.legend()
plt.savefig('../figs/polynomial.png')
plt.show()