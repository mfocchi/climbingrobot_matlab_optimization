function [ineq, eq, number_of_constr, solution_constr] = constraints(x,   p0,  pf,  Fleg_max, Fr_max, mu, jump_clearance)

global     m num_params b  N_dyn FRICTION_CONE  contact_normal  int_method

% ineq are <= 0

Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(num_params+1:num_params+N_dyn); 
Fr_r = x(num_params+N_dyn+1:num_params+2*N_dyn); 


% check they are column vectors
p0 = p0(:);
pf = pf(:);
     
% size not known
ineq = zeros(1,0);

% number of constraints
number_of_constr.wall_constraints = N_dyn;
number_of_constr.retraction_force_constraints = 4*N_dyn; %unilateral and actuation for 2 ropes

if FRICTION_CONE
    number_of_constr.force_constraints  = 3;
else
    number_of_constr.force_constraints  = 2; %unilateral and actuation
end
number_of_constr.initial_final_constraints = 2;
number_of_constr.via_point = 1;


% variable intergration step
dt_dyn = Tf / (N_dyn-1);

% single shooting
x0 =  computeStateFromCartesian(p0);
[~,~,x, t] = integrate_dynamics(x0,0, dt_dyn, N_dyn, Fr_l,Fr_r,Fleg, int_method);
psi = x(1,:);
l1 = x(2,:);
l2 = x(3,:);
psid = x(4,:);
l1d = x(5,:);
l2d = x(6,:);

p = [l1.*sin(psi).*(1 - (b^2 + l1.^2 - l2.^2).^2/(4*b^2*l1.^2)).^(1/2);
    (b^2 + l1.^2 - l2.^2)/(2*b);
    -l1.*cos(psi)*(1 - (b^2 + l1.^2 - l2.^2).^2/(4*b^2*l1.^2)).^(1/2)];

p_0 = p(:, 1);
p_f = p(:,end);
 

solution_constr.p = p;
solution_constr.psi = psi;
solution_constr.l1 = l1;
solution_constr.l2 = l2;
solution_constr.psid = psid;
solution_constr.l1d = l1d;
solution_constr.l2d = l2d;
solution_constr.time = t;
solution_constr.final_error_discrete = norm(p(:,end) - pf);


% 1 -N_dyn  constraint to do not enter the wall, p_x >=0
for i=1:N_dyn 
    ineq = [ineq -p(1,i) ];   
    %ineq = [ineq -psi(i) ];   
    
end 
% % % debug
% disp('after wall')
% length(ineq)

% 2- N_dyn constraints on retraction force   -Fr_max < Fr < 0 
% unilaterality
for i=1:N_dyn     
     ineq = [ineq  Fr_l(i) ]; % Fr_l <0
    
end 
for i=1:N_dyn     
     ineq = [ineq  Fr_r(i) ]; % Fr_r <0
    
end 

% max force
for i=1:N_dyn 
  ineq = [ineq -Fr_max - Fr_l(i)];    % -Fr_max -Fr_l <0
end
for i=1:N_dyn 
  ineq = [ineq -Fr_max - Fr_r(i)];    % -Fr_max -Fr_r <0
end

% debug
% disp('after Fr')
% length(ineq)

% constraints on impulse force
contact_tang_y = cross(cross(contact_normal, [0;1;0]),contact_normal); 
contact_tang_z = cross(cross(contact_normal, [0;0;1]),contact_normal); 
% compute components
Fun = contact_normal'*Fleg;
Futy = contact_tang_y'*Fleg;
Futz = contact_tang_z'*Fleg;
Fut_norm = sqrt(Futy^2 +Futz^2);

Fun_min = 0;

%3 ------------------------------ Fleg constraints

% unilateral
ineq = [ineq  (-Fun + Fun_min)]  ; %(Fun >fmin ) 
%max force 
ineq = [ineq  (norm(Fleg) -Fleg_max)]   ;%(Fun < fun max ) actuation

if FRICTION_CONE
    ineq = [ineq  (Fut_norm -mu*Fun)]; %friction constraints
end
% 
% % debug
% disp('after Fu')
% length(ineq)

% final point  variable slack  
%ineq= [ineq norm(p_f - pf) - x(num_params+N+N_dyn+1)];
% 4- initial final point  fixed slack 
fixed_slack = 0.02;%*norm(p0 - pf); 
ineq= [ineq norm(p_0 - p0) - fixed_slack];
ineq= [ineq norm(p_f - pf) - fixed_slack];


%5 - jump clearance
if number_of_constr.via_point >0 
    ineq = [ineq (-p(1,N_dyn/2) +jump_clearance) ];   
end

eq = [];


if any(isinf(ineq))
    disp('Infn in constraint')
    find(isinf(ineq)) 
    isinf(ineq)
end
if any(isnan(ineq))
    disp('Nan in constraint')
    find(isnan(ineq))
    isnan(ineq)
end

end