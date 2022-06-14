
%
clear all ; close all ; clc
global m l g w1 w2 w3 p0 pf  mu N dt Tf Fun_max theta0 phi0 num_params
m = 1;
l = 4;
g = 9.81;
%pendulum period
T_pend = 2*pi*sqrt(l/g)/4; % half period


% physical limits
Fun_max = 15;
mu = 1.0;
tol = 0.1;

num_params = 4;
w1 = 0.1 % minimize path length
w2 = 1; %red final
w3 = 0.1 ;
N = 10 ; % energy constraints

index_constraints = [];
cost_violation = [];
% 

dt=0.005;     
theta0 = 0.05; %theta0 = 0.523
phi0 = 0 ;
thetaf= 0.8864 ;
phif = 1.5468 ;

p0 = [l*sin(theta0)*cos(phi0); l*sin(theta0)*sin(phi0); -l*cos(theta0)];
pf = [l*sin(thetaf)*cos(phif); l*sin(thetaf)*sin(phif); -l*cos(thetaf)];

x0 = [[0.1,  5,            2.5,   T_pend], zeros(1,N)];
lb = [[ 0,   0 ,      -Fun_max,  0.05], zeros(1,N)]; %slacks should be biggere than zero
ub = [[ 1.0, Fun_max,  Fun_max,    5.], 10*ones(1,N)];

%options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
options = optimoptions('fmincon','Display','iter','Algorithm','sqp', 'MaxIterations', 1500);

[x, final_cost, EXITFLAG] = fmincon(@cost_force,x0,[],[],[],[],lb,ub,@constraints_force, options);
slacks = sum(x(num_params:end));

[E, path_length ] = plot_curve_forceoptim(x, p0, pf,false, false);

