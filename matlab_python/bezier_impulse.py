# -*- coding: utf-8 -*-
"""
Created on Wed May  4 11:19:27 2022

@author: mfocchi
"""

# Resources
# https://pomax.github.io/bezierinfo/
# https://bezier.readthedocs.io/en/stable/python/reference/bezier.curve.html



import matplotlib.pyplot as plt
import numpy as np

# Pascal triangle
lut = [      [1],           # n=0
            [1,1],          # n=1
           [1,2,1],         # n=2
          [1,3,3,1],        # n=3
         [1,4,6,4,1],       # n=4
        [1,5,10,10,5,1],    # n=5
       [1,6,15,20,15,6,1]]  # n=6

#def binomial(n,k):
#    while(n >= lut.length):
#        s = lut.length
#        nextRow[0] = 1
#    for (i=1, prev=s-1; i<s; i++):
#        nextRow[i] = lut[prev][i-1] + lut[prev][i]
#    nextRow[s] = 1
#    lut.append(nextRow)
#return lut[n][k]
#
## n order bezier
#def Bezier(n,t,w[]):
#    sum = 0
#    for(k=0; k<=n; k++):
#        sum += w[k] * binomial(n,k) * (1-t)^(n-k) * t^(k)
#    return sum

# 2 order bezier
def Bezier2(t, weights):
    t2 = t * t
    mt = 1-t
    mt2 = mt * mt

    return weights[0]*mt2 + weights[1]*2*mt*t + weights[2]*t2

def Bezier3(t, weights):
    t2 = t * t
    t3 = t2 * t
    mt = 1-t
    mt2 = mt * mt
    mt3 = mt2 * mt
    
    return weights[0]*mt3 + 3*weights[1]*mt2*t + 3*weights[2]*mt*t2 + weights[3]*t3


def RationalBezier3(t, w ,r):
    t2 = t * t
    t3 = t2 * t
    mt = 1-t
    mt2 = mt * mt
    mt3 = mt2 * mt
    f = [    r[0] * mt3,
        3 * r[1] * mt2 * t,
        3 * r[2] * mt * t2,
        r[3] * t3  ]
  
    basis = f[0] + f[1] + f[2] + f[3]
    return (f[0] * w[0] + f[1] * w[1] + f[2] * w[2] + f[3] * w[3])/basis

    
m = 5 #kg
N = 500
T_f = 2.
T_thrust = 0.2
time = np.linspace(0, T_f, N)
dt = T_f/N
max_force = m*9.81 * 3 

# bezier params
up_bezier_control_points = [0., 0,  1., 1.]
down_bezier_control_points = [1.,1.,  0., 0.]
bezier_cp_influence = [1., 0.4,  0.4, 1.] # relative influence of cp

force = []
    

t = 0.
# first half
while t<(T_thrust/2.):
    #normalize bw 0 and 1
    t_norm = t/(T_thrust/2.)   
    force.append( max_force * RationalBezier3(t_norm, up_bezier_control_points, bezier_cp_influence))
    t+=dt
    
t = 0.
# second half
while t<(T_thrust/2.):
    #normalize bw 0 and 1
    t_norm = t/(T_thrust/2.)      
    force.append( max_force * RationalBezier3(t_norm, down_bezier_control_points, bezier_cp_influence))
    t+=dt
# pad with  zeros till the end
for i in range( int((T_f - T_thrust)/dt) ):
    force.append( 0.)

fig = plt.figure()
plt.plot(time, force, "ob")
plt.xlim(0, T_thrust + 0.1)
plt.ylim(0, max_force)
plt.title("Bezier impulse")
plt.xlabel("time [$s$]")
plt.ylabel("force [$N$]")
plt.grid()
plt.legend()
plt.savefig('../figs/bezier.png')
plt.show()

