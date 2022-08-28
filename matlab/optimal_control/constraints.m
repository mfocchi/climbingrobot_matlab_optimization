function [ineq, eq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints, dynamic_constraints, solution_constr] = constraints(x,   p0,  pf,  Fun_max, Fr_max, mu, fixed_time)

global  g N   m num_params l_uncompressed T_th N_dyn FRICTION_CONE 


% ineq are <= 0

thetad0 = x(1);
phid0 = x(2);
K = x(3);


switch nargin
    case 7
        Tf = fixed_time;
        %fprintf(2, 'constraints: time optim off\n')
    otherwise           
        Tf = x(4);
end

% variable intergration step
dt_dyn = Tf / N_dyn;

fine_index = floor(linspace(1, N_dyn,N));%[1:N_dyn/N:N_dyn];

% check they are column vectors
p0 = p0(:);
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
p_0 = p(:, 1);
p_f = p(:,end);
 

solution_constr.p = p;
solution_constr.theta = theta;
solution_constr.phi = phi;
solution_constr.l = l;
solution_constr.thetad = thetad;
solution_constr.phid = phid;
solution_constr.ld = ld;
solution_constr.time = t;

% number of constraints
energy_constraints = N-1;
wall_constraints = N_dyn;
retraction_force_constraints = 2*N_dyn;
dynamic_constraints = N_dyn-1;
initial_final_constraints = 1;

if FRICTION_CONE
    force_constraints  = 3;
else
    force_constraints  = 2;
end

% size not known
ineq =[];% zeros(1, energy_constraints + wall_constraints +retraction_force_constraints+force_constraints+initial_final_constraints);
    
E = zeros(1,N);
sigma_energy = zeros(1,N);

for i=1:N    
    idx = fine_index(i);
    E(i) = m*l(idx)^2/2*(thetad(idx)^2+sin(theta(idx))^2*phid(idx)^2 ) + m*ld(idx)^2/2 - m*g*l(idx)*cos(theta(idx)) + K*(l(idx)-l_uncompressed).^2/2;
    sigma_energy(i) = x(num_params+i);        
    if (i>=2)
        ineq = [ineq (abs(E(i) - E(i-1)) - sigma_energy(i))];
        %ineq = [ineq 0];
    end

end



% constraint to do not enter the wall, p_x >=0
for i=1:N_dyn 

    ineq = [ineq -p(1,i) ];
    
end 


% constraints on retraction force   -Fr_max < Fr = -K*(l-luncompr) < 0 
Fr = zeros(1,N_dyn);
for i=1:N_dyn 
    
    Fr(i) = -K*(l(i) - l_uncompressed);
    ineq = [ineq  Fr(i) ]; % Fr <0
    
end 
for i=1:N_dyn
 
  ineq = [ineq -Fr_max - Fr(i)];    % -Fr_max -Fr <0
end



%evaluate inpulse ( the integral of the gaussian is 1) 
Fun = m*l_0*thetad0/T_th;
Fut = m*l_0*sin(theta0)*phid0/T_th;

ineq = [ineq  (sqrt(Fun^2 + Fut^2) -Fun_max)]   ;%(Fun < fun max )
ineq = [ineq  (-Fun)]  ; %(Fun >0 )

if FRICTION_CONE
    ineq = [ineq  (abs(Fut) -mu*Fun)]; %friction constraints
end



%dynamic constraints
sigma_dyn = zeros(1, N_dyn);
for i=1:N_dyn   
    sigma_dyn(i) = x(num_params + N +i);        
    if (i>=2)
        xk = states(:,i);
        xk1 =states(:,i-1);
        ineq = [ineq (norm(xk - xk1 -  dt_dyn* dynamics_autonomous(xk1, K)) - sigma_dyn(i))];
        
    end
end


% final point   
ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)];



eq = [];
%energy as equality (w3 <= 10 otherwise it does not converge)
% for i=1:N    
%     sigma_energy(i) = x(num_params+i);  
%     idx = fine_index(i);
%     E(i) = m*l(idx)^2/2*(thetad(idx)^2+sin(theta(idx))^2*phid(idx)^2 ) + m*ld(idx)^2/2 - m*g*l(idx)*cos(theta(idx)) + K*(l(idx)-l_uncompressed).^2/2; 
%     if (i>=2)
%         eq = [eq E(i) - E(i-1) - sigma_energy(i)];        
%     end
% end

if any(isinf(ineq))
    ineq
end


end