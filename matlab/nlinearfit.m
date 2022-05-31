


%
clear all ; close all ; clc
global m l g  w1 w2 w3 p0 pf N time OLD_FORMULATION POLY_TYPE num_params
m = 1;
l = 4;
g = 9.81;
Tf = 1.0;

w1 = 1 ; % green initial
w2 = 0.6; %red final
w3 = 0.1 ;

N = 10 ;
OLD_FORMULATION = 1;
POLY_TYPE = 0  % 0 cubic, 1 quintic

if (POLY_TYPE)
    num_params = 12;
else 
    num_params = 8; 
end

time = linspace(0, Tf, N) ;
theta0 = pi/6 ; %theta0 = 0.523
phi0 = 0 ;
thetaf= 0.8864 ;
phif = 1.5468 ;

p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

x0 = [0.1*ones(1,num_params), zeros(1,N)] ;
lb = [-10*ones(1,num_params), zeros(1,N)];
ub = [10*ones(1,num_params), 10*ones(1,N)];

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

[x, final_cost] = fmincon(@cost,x0,[],[],[],[],lb,ub,@constraints, options)
slacks = sum(x(9:end))


plot_curve(x, Tf, p0, pf);
    



