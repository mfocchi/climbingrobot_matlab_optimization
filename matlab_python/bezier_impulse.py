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
    #time needs to be normalized bw 0 and 1
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



# example with  Ricdcardo     

N = 1000
T_thrust = 0.2
time = np.linspace(0, T_thrust, N)
dt = T_thrust/N


# bezier params for a 3rd order bezier
bezier_control_pointsX = [0., 0.0, 1,  10.5]
bezier_control_pointsY = [0.,  0.0, 1,  10.5]
bezier_control_pointsZ = [0.,  0.0, 1,   10.5]

# bezier derivativre params, hte derivative is a n-1th curve with weights w0', .. wn-1' = n*(wi+1 - wi)
order_bezier = 3
bezier_control_pointsX_der = [0]*3
bezier_control_pointsY_der = [0]*3
bezier_control_pointsZ_der = [0]*3
bezier_control_pointsX_der[0] = order_bezier*(bezier_control_pointsX[1] - bezier_control_pointsX[0])
bezier_control_pointsX_der[1] = order_bezier*(bezier_control_pointsX[2] - bezier_control_pointsX[1])
bezier_control_pointsX_der[2] = order_bezier*(bezier_control_pointsX[3] - bezier_control_pointsX[2])

bezier_control_pointsY_der[0] = order_bezier*(bezier_control_pointsY[1] - bezier_control_pointsY[0])
bezier_control_pointsY_der[1] = order_bezier*(bezier_control_pointsX[2] - bezier_control_pointsY[1])
bezier_control_pointsY_der[2] = order_bezier*(bezier_control_pointsY[3] - bezier_control_pointsY[2])

bezier_control_pointsZ_der[0] = order_bezier*(bezier_control_pointsZ[1] - bezier_control_pointsZ[0])
bezier_control_pointsZ_der[1] = order_bezier*(bezier_control_pointsZ[2] - bezier_control_pointsZ[1])
bezier_control_pointsZ_der[2] = order_bezier*(bezier_control_pointsZ[3] - bezier_control_pointsZ[2])

    
bezier_control_pointsX_2der = [0]*2
bezier_control_pointsY_2der = [0]*2
bezier_control_pointsZ_2der = [0]*2

bezier_control_pointsX_2der[0] = 2*(bezier_control_pointsX_der[1] - bezier_control_pointsX_der[0])
bezier_control_pointsX_2der[1] = 2*(bezier_control_pointsX_der[2] - bezier_control_pointsX_der[1])

bezier_control_pointsY_2der[0] = 2*(bezier_control_pointsY_der[1] - bezier_control_pointsY_der[0])
bezier_control_pointsY_2der[1] = 2*(bezier_control_pointsY_der[2] - bezier_control_pointsY_der[1])

bezier_control_pointsZ_2der[0] = 2*(bezier_control_pointsZ_der[1] - bezier_control_pointsZ_der[0])
bezier_control_pointsZ_2der[1] = 2*(bezier_control_pointsZ_der[2] - bezier_control_pointsZ_der[1])


pos =  np.empty((3,N))
posd =  np.empty((3,N))
posdd =  np.empty((3,N))
posd_bezier =  np.empty((3,N))
posdd_bezier =  np.empty((3,N))

com0 = np.array([0,0,1.])
comf = np.array([0.1,0,1.1])
pos_old = np.zeros((3))
posd_old = np.zeros((3))  

t = 0.
i = 0
# first half
while t<(T_thrust):
    #time needs to be normalized bw 0 and 1
    t_norm = t/(T_thrust)
    pos[:,i] = np.array([  Bezier3(t_norm, bezier_control_pointsX), Bezier3(t_norm, bezier_control_pointsY), Bezier3(t_norm, bezier_control_pointsZ)])
    posd[:,i] = (pos[:,i] - pos_old)/dt
    posdd[:,i] = (posd[:,i] - posd_old)/dt
    pos_old = pos[:,i]
    posd_old = posd[:,i]
    # Note you need to scale the derivative of the bezier if you are using a different time frame than 0 and 1 xdot = dP/dt = dP/dt' * dt'/dt where dt' in [0 ,1] dt in [ 0, Tthust] 
    posd_bezier[:,i] = 1/T_thrust* np.array([  Bezier2(t_norm, bezier_control_pointsX_der), Bezier2(t_norm, bezier_control_pointsY_der), Bezier2(t_norm, bezier_control_pointsZ_der)])
    posdd_bezier[:,i] =  1/(T_thrust*T_thrust) * ( bezier_control_pointsZ_2der[0] *(1-t_norm) + bezier_control_pointsZ_2der[1]*t_norm)
    t+=dt
    i+=1
    

fig = plt.figure()
plt.subplot(3,1,1)
plt.title("trhusting traj")
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posx ")
plt.plot(time, pos[0,:], "ob")
plt.grid()

plt.subplot(3,1,2)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posy ")
plt.plot(time, pos[1,:], "ob")
plt.grid()

plt.subplot(3,1,3)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posz")
plt.plot(time, pos[2,:], "ob")
plt.grid()
plt.show()

fig = plt.figure()
plt.subplot(3,1,1)
plt.title("trhusting traj der")
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdx ")
plt.plot(time, posd[0,:], "ob")
plt.plot(time, posd_bezier[0,:], "or")
plt.grid()

plt.subplot(3,1,2)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdy ")
plt.plot(time, posd[1,:], "ob")
plt.plot(time, posd_bezier[1,:], "or")
plt.grid()

plt.subplot(3,1,3)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdz")
plt.plot(time, posd[2,:], "ob")
plt.plot(time, posd_bezier[2,:], "or")
plt.grid()
plt.show()

fig = plt.figure()
plt.subplot(3,1,1)
plt.title("trhusting traj der")
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdx ")
plt.plot(time, posdd[0,:], "ob")
plt.plot(time, posdd_bezier[0,:], "or")
plt.grid()

plt.subplot(3,1,2)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdy ")
plt.plot(time, posdd[1,:], "ob")
plt.plot(time, posdd_bezier[1,:], "or")
plt.grid()

plt.subplot(3,1,3)
plt.xlim(0, T_thrust)
plt.xlabel("time [$s$]")
plt.ylabel("posdz")
plt.plot(time, posdd[2,:], "ob")
plt.plot(time, posdd_bezier[2,:], "or")
plt.grid()

plt.show()