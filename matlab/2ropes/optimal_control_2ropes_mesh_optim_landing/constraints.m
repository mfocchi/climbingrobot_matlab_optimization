function [ineq, eq, number_of_constr, solution_constr] = constraints(x,   p0,  pf,  Fleg_max, Fr_max, mu, params)


% ineq are <= 0
Fleg = [ x(1); x(2); x(3)];
Tf = x(4);
Fr_l = x(params.num_params+1:params.num_params+params.N_dyn);
Fr_r = x(params.num_params+params.N_dyn+1:params.num_params+2*params.N_dyn);


% check they are column vectors
p0 = p0(:);
pf = pf(:);

% size not known
ineq = zeros(1,0);

% number of constraints
number_of_constr.wall_constraints = params.N_dyn;
number_of_constr.retraction_force_constraints = 0;% already included in bounds %4*N_dyn; %unilateral and actuation for 2 ropes

if params.FRICTION_CONE
    number_of_constr.force_constraints  = 3;
else
    number_of_constr.force_constraints  = 2; %unilateral and actuation
end
number_of_constr.final_constraints = 5;

if  strcmp(   params.obstacle_avoidance, 'mesh')
    number_of_constr.via_point = 1;
else
    number_of_constr.via_point = 0;
end


% variable intergration step
dt_dyn = Tf / (params.N_dyn-1);


% single shooting
state0 =  computeStateFromCartesian(params, p0);
[states, t] = computeRollout(state0, 0,dt_dyn, params.N_dyn, Fr_l, Fr_r,Fleg,params.int_method,params.int_steps,params);
psi = states(1,:);
l1 = states(2,:);
l2 = states(3,:);
psid = states(4,:);
l1d = states(5,:);
l2d = states(6,:);
p = computePositionVelocity(params, psi, l1, l2); %only position

% I assume px py pz  are row vectors
p_0 = p(:, 1);
p_f = p(:,end);

% init struct foc C++ code generation
solution = struct;
solution_constr.p = p;
solution_constr.psi = psi;
solution_constr.l1 = l1;
solution_constr.l2 = l2;
solution_constr.psid = psid;
solution_constr.l1d = l1d;
solution_constr.l2d = l2d;
solution_constr.time = t;




% 1 -N_dyn  constraint to do not enter the wall, p_x >=0

if strcmp(   params.obstacle_avoidance, 'none')
    for i=1:params.N_dyn
        ineq = [ineq -p(1,i) ];
    end

elseif strcmp(params.obstacle_avoidance, 'mesh')
    %p_x > wall_x + jump_clearance => p_x -wall_z- jump_clearance  >0 => -p_x +wall_z + jump_clearance  <0
    for i=1:params.N_dyn
        wall_x = wallSurfaceEval(p(3, i), p(2, i),params);
        ineq = [ineq (-p(1,i)+wall_x) ];
        %fprintf('debug wall: %f %f \n',p(1,i),params.jump_clearance +wall_x);
    end
else
    disp('wrong ostacle')

end

% % % debug
% disp('after wall')
% length(ineq)

% 2- N_dyn constraints on retraction force   -Fr_max < Fr < 0
% unilaterality

if number_of_constr.retraction_force_constraints>0
    for i=1:params.N_dyn
        ineq = [ineq  Fr_l(i) ]; % Fr_l <0

    end
    for i=1:params.N_dyn
        ineq = [ineq  Fr_r(i) ]; % Fr_r <0

    end

    % max force    -Fr>-Fr_max
    for i=1:params.N_dyn
        ineq = [ineq (-Fr_max - Fr_l(i))];    % -Fr_max -Fr_l <0
    end
    for i=1:params.N_dyn
        ineq = [ineq (-Fr_max - Fr_r(i))];    % -Fr_max -Fr_r <0
    end
end

% debug
% disp('after Fr')
% length(ineq)

% constraints on impulse force
contact_tang_y = cross(cross(params.contact_normal(:), [0;1;0]),params.contact_normal(:));
contact_tang_z = cross(cross(params.contact_normal(:), [0;0;1]),params.contact_normal(:));
% compute components
Fun = params.contact_normal(:)'*Fleg;
Futy = contact_tang_y'*Fleg;
Futz = contact_tang_z'*Fleg;
Fut_norm = sqrt(Futy^2 +Futz^2);

Fun_min = 0;

%3 ------------------------------ Fleg constraints

% unilateral
ineq = [ineq  (-Fun + Fun_min)]  ; %(Fun >fmin )
%max force
ineq = [ineq  (norm(Fleg) -Fleg_max)]   ;%(|Fun| < fun max ) actuation

if params.FRICTION_CONE %|Fut| < mu*Fun
    ineq = [ineq  (Fut_norm -mu*Fun)]; %friction constraints
end

%4- landing constraint, landing point inside the selected patch
y_min = pf(2)-params.patch_side/2;
y_max = pf(2)+params.patch_side/2;
z_min = pf(3)-params.patch_side/2;
z_max = pf(3)+params.patch_side/2;

%constraint Y direction inside patch  bounds 
%p_f(2) < ymax => p_f(2) -ymax < 0
ineq= [ineq (p_f(2)-y_max) ];
%p_f(2) > ymin => -p_f(2) < -ymin => -p_f(2) + ymin<0
ineq= [ineq (-p_f(2)+y_min) ];

%constraint Z direction inside patch  bounds
%p_f(3) < zmax => p_f(3) -zmax < 0
ineq= [ineq (p_f(3)-z_max)];
%p_f(3) > zmin => -p_f(3) < -zmin => -p_f(3) + zmin<0
ineq= [ineq (-p_f(3)+z_min)];

fixed_slack = 0.02;
%constraint the X (otherwise it finds something in the air!)
wall_x_min = wallSurfaceEval(p_f(3), p_f(2),params);
ineq = [ineq (norm(p_f(1)-wall_x_min)-fixed_slack) ];

%old way 
% ineq= [ineq (norm(p_f - pf) - fixed_slack)];
% number_of_constr.final_constraints =  1;

%5 - jump clearance p_x > jump_clearance
if number_of_constr.via_point >0
    ineq = [ineq (-p(1,params.N_dyn/2) +params.jump_clearance) ];

end

eq = [];


% if any(isinf(ineq))
%     disp('Infn in constraint')
%     find(isinf(ineq))
%     isinf(ineq)
% end
% if any(isnan(ineq))
%     disp('Nan in constraint')
%     find(isnan(ineq))
%     isnan(ineq)
% end

end