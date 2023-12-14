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


class BezierLanding:
    def __init__(self, dt = 0.004):

        # Pascal triangle
        self.lut = [      [1],           # n=0
                    [1,1],          # n=1
                   [1,2,1],         # n=2
                  [1,3,3,1],        # n=3
                 [1,4,6,4,1],       # n=4
                [1,5,10,10,5,1],    # n=5
               [1,6,15,20,15,6,1]]  # n=6

        self.order_bezier = 3
        self.bezier_control_pointsZ = [0]*(self.order_bezier + 1)
        self.bezier_control_pointsZ_der = [0] * self.order_bezier
        self.bezier_control_pointsZ_2der = [0]*(self.order_bezier-1)
        self.dt = dt
        max_N =int(2./dt)
        self.time = np.linspace(0, 2., max_N)
        self.pos = np.zeros((max_N))
        self.posd = np.zeros((max_N))
        self.posdd = np.zeros((max_N))

    def binomial(self, n,k):
        return self.lut[n][k]

    # n order bezier
    def Bezier(self, n,t,w):
       sum = 0
       for k in range(n+1):
           sum += w[k] * self.binomial(n,k) * (1-t)**(n-k) * t**(k)
       return sum

    def Bezier2(self, t, weights):
        t2 = t * t
        mt = 1 - t
        mt2 = mt * mt
        return weights[0] * mt2 + weights[1] * 2 * mt * t + weights[2] * t2


    def Bezier3(self, t, weights):
        t2 = t * t
        t3 = t2 * t
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        return weights[0] * mt3 + 3 * weights[1] * mt2 * t + 3 * weights[2] * mt * t2 + weights[3] * t3


    def computeBezier(self, p0, pf, v0, a_max, af, dt):
        # bezier params for a 3rd order bezier
        # compute Tth from condition pdd(0) = a_max
        T_thrust =( -3 *v0 + np.sqrt(9*v0**2   + 12 *( a_max + 0.5 *af)*(pf-p0))  )    / (2*a_max +af)
        self.bezier_control_pointsZ[0] = p0 # p(0) = p0
        self.bezier_control_pointsZ[1] = p0 + v0*T_thrust/3 # pd(0) = v0
        self.bezier_control_pointsZ[2] = (pf  + p0)/2 + v0*T_thrust/6  - af/12*T_thrust**2# pdd(T_thrust) = af
        self.bezier_control_pointsZ[3] = pf # p(f) = pf

        # bezier derivativre params, hte derivative is a n-1th curve with weights w0', .. wn-1' = n*(wi+1 - wi)
        self.bezier_control_pointsZ_der[0] = self.order_bezier*(self.bezier_control_pointsZ[1] - self.bezier_control_pointsZ[0])
        self.bezier_control_pointsZ_der[1] = self.order_bezier*(self.bezier_control_pointsZ[2] - self.bezier_control_pointsZ[1])
        self.bezier_control_pointsZ_der[2] = self.order_bezier*(self.bezier_control_pointsZ[3] - self.bezier_control_pointsZ[2])

        self.bezier_control_pointsZ_2der[0] = 2*(self.bezier_control_pointsZ_der[1] - self.bezier_control_pointsZ_der[0])
        self.bezier_control_pointsZ_2der[1] = 2*(self.bezier_control_pointsZ_der[2] - self.bezier_control_pointsZ_der[1])

        N = int(T_thrust /self.dt)+1
        for i in range(N):
            #time needs to be normalized bw 0 and 1
            t_norm = self.time[i]/(T_thrust)
            self.pos[i] =  self.Bezier3(t_norm, self.bezier_control_pointsZ)
            self.posd[i] = 1/T_thrust* self.Bezier2(t_norm, self.bezier_control_pointsZ_der)
            self.posdd[i] = 1 / (T_thrust * T_thrust) * (self.bezier_control_pointsZ_2der[0] * (1 - t_norm) + self.bezier_control_pointsZ_2der[1] * t_norm)

        return self.time[:N], self.pos[:N], self.posd[:N], self.posdd[:N]