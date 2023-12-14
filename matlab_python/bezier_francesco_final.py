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
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
# Pascal triangle
lut = [      [1],           # n=0
            [1,1],          # n=1
           [1,2,1],         # n=2
          [1,3,3,1],        # n=3
         [1,4,6,4,1],       # n=4
        [1,5,10,10,5,1],    # n=5
       [1,6,15,20,15,6,1]]  # n=6

def binomial(n,k):
    return lut[n][k]

# n order bezier
def Bezier(n,t,w):
   sum = 0
   for k in range(n+1):
       sum += w[k] * binomial(n,k) * (1-t)**(n-k) * t**(k)
   return sum

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
    # the ratio value gives an idea of how strong each control point influences the curve higher the ratio the closer the curve to that point
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



N = 1000




pos =  np.empty((N))
posd =  np.empty((N))
posdd =  np.empty((N))

p0 = 0.25
pf = 0.25
v0 = -1
a_max = 500 / 20 *0.25

# bezier params for a 3rd order bezier
T_thrust =( -3 *v0 + np.sqrt(9*v0**2   + 12 *a_max*(pf-p0))  )    / (2*a_max)

bezier_control_pointsZ = [0]*4
bezier_control_pointsZ[0] = p0
bezier_control_pointsZ[1] = p0 + v0*T_thrust/3
bezier_control_pointsZ[2] = (pf  + p0)/2 + v0*T_thrust/6
bezier_control_pointsZ[3] = pf

# bezier derivativre params, hte derivative is a n-1th curve with weights w0', .. wn-1' = n*(wi+1 - wi)
order_bezier = 3
bezier_control_pointsZ_der = [0]*3
bezier_control_pointsZ_der[0] = order_bezier*(bezier_control_pointsZ[1] - bezier_control_pointsZ[0])
bezier_control_pointsZ_der[1] = order_bezier*(bezier_control_pointsZ[2] - bezier_control_pointsZ[1])
bezier_control_pointsZ_der[2] = order_bezier*(bezier_control_pointsZ[3] - bezier_control_pointsZ[2])


bezier_control_pointsZ_2der = [0]*2
bezier_control_pointsZ_2der[0] = 2*(bezier_control_pointsZ_der[1] - bezier_control_pointsZ_der[0])
bezier_control_pointsZ_2der[1] = 2*(bezier_control_pointsZ_der[2] - bezier_control_pointsZ_der[1])

time = np.linspace(0, T_thrust, N)
dt = T_thrust/N
t = 0.
i = 0
# first half
while i<N:
    #time needs to be normalized bw 0 and 1
    t_norm = t/(T_thrust)
    pos[i] =  Bezier3(t_norm, bezier_control_pointsZ)
    # Note you need to scale the derivative of the bezier if you are using a different time frame than 0 and 1 xdot = dP/dt = dP/dt' * dt'/dt where dt' in [0 ,1] dt in [ 0, Tthust]
    posd[i] = 1/T_thrust* Bezier2(t_norm, bezier_control_pointsZ_der)
    posdd[ i] = 1 / (T_thrust * T_thrust) * (bezier_control_pointsZ_2der[0] * (1 - t_norm) + bezier_control_pointsZ_2der[1] * t_norm)
    t+=dt
    i+=1

# #5th order
# af = 9.81
# a0 = 7.0
# bezier_control_points4Z = [0]*5
# bezier_control_points4Z[0]  = p0
# bezier_control_points4Z[1]  = p0 + v0/4*T_thrust
# bezier_control_points4Z[3]  = pf - vf/4*T_thrust
# bezier_control_points4Z[4]  = pf
# #bezier_control_points4Z[2]  = af*T_thrust**2/12. - pf + 2*bezier_control_points4Z[3]
# bezier_control_points4Z[2]  = a0*T_thrust**2/12. - p0 + 2*bezier_control_points4Z[1]
#
#
# bezier_control_points4Z_der =  [0]*4
# bezier_control_points4Z_der[0] = order_bezier*(bezier_control_points4Z[1] - bezier_control_points4Z[0])
# bezier_control_points4Z_der[1] = order_bezier*(bezier_control_points4Z[2] - bezier_control_points4Z[1])
# bezier_control_points4Z_der[2] = order_bezier*(bezier_control_points4Z[3] - bezier_control_points4Z[2])
# bezier_control_points4Z_der[3] = order_bezier*(bezier_control_points4Z[4] - bezier_control_points4Z[3])
#
# t = 0.
# i = 0
#
# while t<(T_thrust):
#     #time needs to be normalized bw 0 and 1
#     t_norm = t/(T_thrust)
#     pos[i] = Bezier(4, t_norm, bezier_control_points4Z)
#     posd_bezier[i] = 1 / T_thrust * Bezier(3, t_norm, bezier_control_points4Z_der)
#     t+=dt
#     i+=1
    
#
# fig = plt.figure()
#
# plt.title("trhusting traj")
# plt.xlim(0, T_thrust)
# plt.xlabel("time [$s$]")
# plt.ylabel("posx ")
# plt.plot(time, pos[:], "ob")
# plt.grid()
#
# fig = plt.figure()
# plt.title("trhusting traj der")
# plt.xlim(0, T_thrust)
# plt.xlabel("time [$s$]")
# plt.ylabel("posdx ")
# plt.plot(time, posd[:], "or")
# plt.grid()
#
# fig = plt.figure()
# plt.title("trhusting traj der")
# plt.xlim(0, T_thrust)
# plt.xlabel("time [$s$]")
# plt.ylabel("posddx ")
# plt.plot(time, posdd[:], "or")
# plt.grid()
#
# plt.show()


from bezier_landing import BezierLanding
b = BezierLanding(dt = 0.004)
import time as t
tic = t.time()
p0 = 0.25
pf = 0.25

height = 2
delta_t = np.sqrt(2*height/9.81)
v0 = -delta_t * 9.81
af =500 / 13
a_max = 500 / 13
time_c, pos_c, posd_c, posdd_c  = b.computeBezier(p0, pf, v0, a_max, af, 0.004)
toc = t.time()
#print(toc-tic)
print('initial vel: ', v0)
print('final vel: ', posd_c[-1])
print('thrust duration: ', time_c[-1])
fig = plt.figure()

plt.title("trhusting traj")
plt.xlim(0, time_c[-1])
plt.xlabel("time [$s$]")
plt.ylabel("posx ")
plt.plot(time_c, pos_c[:], "ob")
plt.grid()

fig = plt.figure()
plt.title("trhusting traj der")
plt.xlim(0, time_c[-1])
plt.xlabel("time [$s$]")
plt.ylabel("posdx ")
plt.plot(time_c, posd_c[:], "or")
plt.grid()

fig = plt.figure()
plt.title("trhusting traj der")
plt.xlim(0, time_c[-1])
plt.xlabel("time [$s$]")
plt.ylabel("posddx ")
plt.plot(time_c, posdd_c[:], "or")
plt.grid()

plt.show()