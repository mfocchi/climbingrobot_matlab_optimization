function [ineq, eq, energy_constraints,wall_constraints, retraction_force_constraints, force_constraints, initial_final_constraints] = constraints(x,   p0,  pf,  Fun_max, Fr_max, mu)

global  g N   m num_params l_uncompressed

% ineq are <= 0
Tf = x(1);
time =  linspace(0, Tf, N) ;
a_10 = x(2);
a_11 = x(3);
a_12 = x(4);
a_13 = x(5);
a_20 = x(6);
a_21 = x(7);
a_22 = x(8);
a_23 = x(9);
a_30 = x(10);
a_31 = x(11);
a_32 = x(12);
a_33 = x(13);
K = x(14);
% 

% check they are column vectors
pf = pf(:);
p0 = p0(:);

% parametrizzation with sin theta sing phi
theta = a_10 + a_11*time + a_12*time.^2 +  a_13*time.^3;
thetad =  a_11 + 2*a_12*time + 3*a_13*time.^2;
thetadd = 2*a_12 + 6*a_13*time;

phi = a_20 + a_21*time + a_22*time.^2 + a_23*time.^3;
phid =  a_21 + 2*a_22*time  + 3*a_23*time.^2;
phidd =   2*a_22 + 6*a_23*time;

    
l = a_30 + a_31*time + a_32*time.^2 + a_33*time.^3;
ld =  a_31 + 2*a_32*time  + 3*a_33*time.^2;
ldd =   2*a_32 + 6*a_33*time;

p = [l.*sin(theta).*cos(phi); l.*sin(theta).*sin(phi); -l.*cos(theta)];
p_0 = p(:,1);
p_f = p(:,end);
l_f = l(end);

% number of constraints
energy_constraints = N-1;
wall_constraints = N;
retraction_force_constraints = 2*N;
force_constraints  = 3;
initial_final_constraints = 3;


% size not known
ineq = zeros(1, energy_constraints + wall_constraints +retraction_force_constraints+force_constraints+initial_final_constraints);
    
E = zeros(1,N);
sigma = zeros(1,N);

for i=1:N    % these are 8 constraints 
    
    E(i) = m*l(i)^2/2*(thetad(i)^2+sin(theta(i))^2*phid(i)^2 ) + m*ld(i)^2/2 - m*g*l(i)*cos(theta(i)) + K*(l(i)-l_uncompressed).^2/2;
    sigma(i) = x(num_params+i);        
    if (i>=2)
        ineq = [ineq (abs(E(i) - E(i-1)) - sigma(i))];
    end

end



% constraint to do not enter the wall, p_x >=0
for i=1:N 
    ineq = [ineq -p(1,i) ];
end 

% constraints on retraction force   -Fr_max < Fr = -K*(l-luncompr) < 0 
Fr = zeros(1,N);
for i=1:N 
    Fr(i) = -K*(l(i) - l_uncompressed);
    ineq = [ineq  Fr(i) ]; % Fr <0
    
end 
for i=1:N
    ineq = [ineq -Fr_max - Fr(i)];    % -Fr_max -Fr <0
end

 
[Fun , Fut] = evaluate_initial_impulse(x);


ineq = [ineq  (sqrt(Fun^2 + Fut^2) -Fun_max)]   ;
ineq = [ineq  (-Fun)]  ;
ineq = [ineq  (abs(Fut) -mu*Fun)];


% initial final point
ineq= [ineq norm(p_0 - p0) - x(num_params+N+1)];
ineq= [ineq norm(p_f - pf) - x(num_params+N+2)];
ineq= [ineq abs(norm(pf) - l_f) - x(num_params+N+3)];


eq = [];

% not ok this is too restrictive
% eq= [eq norm(p_0 - p0)  ];
% eq= [eq norm(p_f - pf) ];
% eq= [eq abs(norm(pf) - l_f)  ];

if any(isinf(ineq))
    ineq
end


end