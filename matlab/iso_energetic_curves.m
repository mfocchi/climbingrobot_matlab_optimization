syms l g a_10 a_11 a_12 a_13 a_20 a_21 a_22  a_23 t


% assume theta moves as cubics
theta(t) = a_10 + a_11*t + a_12*t^2 +  a_13*t^3
thetad(t) =  a_11 + 2*a_12*t + 3*a_13*t^2
thetadd(t) = 2*a_12 + 6*a_13*t

phi(t) = a_20 + a_21*t + a_22*t^2
phid(t) =  a_21 + 2*a_22*t  + 3*a_23*t^2
phidd(t) =   2*a_22 + 6*a_23*t


x(t) = l*sin(theta)*cos(phi)
y(t) = l*sin(theta)*sin(phi)
z(t) = -l*cos(theta)

p(t) = [x;y;z]


l = 4
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
eq3 = l*( thetad(t)*thetadd(t) + 2*sin(theta(t))*cos(theta(t))*thetad(t)^2 + sin(theta(t))^2 * thetad(t)*thetadd(t) ) + g*sin(theta(t)) == 0 


[a_10, a_11, a_12, a_13, a_20, a_21, a_22, a_23] = solve(eq1,eq2,eq3, [a_10, a_11, a_12, a_13, a_20, a_21, a_22, a_23])




