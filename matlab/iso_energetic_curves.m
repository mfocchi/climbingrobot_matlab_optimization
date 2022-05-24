syms l g a_10 a_11 a_12 a_13 a_20 a_21 a_22  a_23 t


% assume theta moves as cubics
theta(t) = a_10 + a_11*t + a_12*t^2 +  a_13*t^3
thetad(t) =  a_11 + 2*a_12*t + 3*a_13*t^2
thetadd(t) = 2*a_12 + 6*a_13*t

phi(t) = a_20 + a_21*t + a_22*t^2 + a_23*t^3
phid(t) =  a_21 + 2*a_22*t  + 3*a_23*t^2
phidd(t) =   2*a_22 + 6*a_23*t


x(t) = l*sin(theta)*cos(phi)
y(t) = l*sin(theta)*sin(phi)
z(t) = -l*cos(theta)

p(t) = [x;y;z]


l = 7
theta_f = 0.5
T_f = 2
% got 3 constraints form target

p_tg = [0;l*sin(theta_f);-l*cos(theta_f)]

% got 3 constraints from the  initial point
p_0 = [0;0;-l]


%equations
eq1= (p(0) -p_0) == 0   
eq2= (p(T_f) -p_tg) == 0   
% iso energetic constraint
%eq3 = l*( thetad(t)*thetadd(t) + 2*sin(theta(t))*cos(theta(t))*thetad(t)^2 + sin(theta(t))^2 * thetad(t)*thetadd(t) ) + g*sin(theta(t)) == 0 


[a_10, a_11, a_12, a_13, a_20, a_21, a_22, a_23] = solve(eq1,eq2, [a_10, a_11, a_12, a_13, a_20, a_21, a_22, a_23])



%
clc
global l g  p0 pf N Nbound_constr Nconstr 
l = 4;
g = 9.81;
Tf = 1.0;
N = 10
Nconstr = 11;
Nbound_constr = 3

time_span = linspace(0, Tf, N);

beta0 = ones(1,8);

beta0 = [0.1, 0.1, 0.1, 0.1,  0.1, 0.1, 0.1, 0.1 ] ;
p0 = [0.199; 0; -3.995]     ;
pf =  [0.0; 0.76; -3.92];


Y = [zeros(Nbound_constr, 1) ;  zeros(Nconstr-Nbound_constr,1)];
X = [0; Tf; Tf; time_span(2:end-1)'] ;
W = [1 1  1, ones(1,Nconstr- Nbound_constr)]';

[beta, residuals] = nlinfit(X,Y,@myfun,beta0, 'Weights',W)


%eval trajectory
a_10 = beta(1);
a_11 = beta(2);
a_12 = beta(3);
a_13 = beta(4);
a_20 = beta(5);
a_21 = beta(6);
a_22 = beta(7);
a_23 = beta(8);
t = linspace(0, Tf, 1000)

theta = a_10 + a_11*t + a_12*t.^2 +  a_13*t.^3
thetad =  a_11 + 2*a_12*t + 3*a_13*t.^2
thetadd = 2*a_12 + 6*a_13*t

phi = a_20 + a_21*t + a_22*t.^2 + a_23*t.^3
phid =  a_21 + 2*a_22*t  + 3*a_23*t.^2
phidd =   2*a_22 + 6*a_23*t

p = [l*sin(theta).*cos(phi); l*sin(theta).*sin(phi); -l*cos(theta)] ;   
    
plot3(p(1,:), p(2,:), p(3,:))      
grid on


p(:,1) 
p0

p(:,end) 
pf

norm(pf - p(:,end) )
% 
% S = load('reaction');
% X = S.reactants;
% y = S.rate;
% beta0 = S.beta;
% beta = nlinfit(X,y,@example,beta0)
% 
% 
% 
