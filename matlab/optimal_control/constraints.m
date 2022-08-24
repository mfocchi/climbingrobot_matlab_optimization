function [ineq, eq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints] = constraints(x,   p0,  pf,  Fun_max, Fr_max, mu)

global  g N   m num_params l_uncompressed T_th N_dyn dt_dyn

% ineq are <= 0

thetad0 = x(1);
phid0 = x(2);
K = x(3);
% Tf = x(4);

coarse_index = floor(linspace(1, N_dyn, N));

% check they are column vectors
pf = pf(:);

[theta0, phi0, l_0] = computePolarVariables(p0);
state0 = [theta0, phi0, l_0, thetad0, phid0, 0];
[states, t] = integrate_dynamics(state0,dt_dyn, N_dyn, K);

theta = states(1,:);
phi = states(2,:);
l = states(3,:);
thetad = states(4,:);
phid = states(5,:);
ld = states(6,:);

p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
p_f = p(:,end);
 

% number of constraints
energy_constraints = N-1;
wall_constraints = N;
retraction_force_constraints = 2*N;
force_constraints  = 3;
initial_final_constraints = 1;


% size not known
ineq =[];% zeros(1, energy_constraints + wall_constraints +retraction_force_constraints+force_constraints+initial_final_constraints);
    
E = zeros(1,N);
sigma = zeros(1,N);
% 
% for i=1:N    
%     idx = coarse_index(i);
%     E(idx) = m*l(idx)^2/2*(thetad(idx)^2+sin(theta(i))^2*phid(idx)^2 ) + m*ld(i)^2/2 - m*g*l(idx)*cos(theta(i)) + K*(l(idx)-l_uncompressed).^2/2;
%     sigma(i) = x(num_params+i);        
%     if (i>=2)
%         ineq = [ineq (abs(E(idx) - E(idx-1)) - sigma(i))];
%     end
% 
% end



% constraint to do not enter the wall, p_x >=0
for i=1:N 
    idx = coarse_index(i);
    ineq = [ineq -p(1,idx) ];
end 

% constraints on retraction force   -Fr_max < Fr = -K*(l-luncompr) < 0 
Fr = zeros(1,N);
for i=1:N 
    idx = coarse_index(i);
    Fr(idx) = -K*(l(idx) - l_uncompressed);
    ineq = [ineq  Fr(idx) ]; % Fr <0
    
end 
for i=1:N
    idx = coarse_index(i);
    ineq = [ineq -Fr_max - Fr(idx)];    % -Fr_max -Fr <0
end

%evaluate inpulse ( the integral of the gaussian is 1) 
Fun = m*l_0*thetad0/T_th;
Fut = m*l_0*sin(theta0)*phid0/T_th;

ineq = [ineq  (sqrt(Fun^2 + Fut^2) -Fun_max)]   ;
ineq = [ineq  (-Fun)]  ;
ineq = [ineq  (abs(Fut) -mu*Fun)];


% initial final point
ineq= [ineq norm(p_f - pf) - x(num_params+N+1)];



eq = [];


if any(isinf(ineq))
    ineq
end


end