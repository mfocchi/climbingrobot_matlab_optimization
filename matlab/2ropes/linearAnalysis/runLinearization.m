
% Linearization function for nonlinear centroidal dynamics model x_dot = f(x,u,footPos)

%% Symbolic variables in Casadi
import casadi.*
x = MX.sym('x',12,1) ;  % States
u = MX.sym('u',2,1);   % Inputs
Icom=MX.sym('Icom',9,1); % Inertia
x_h=MX.sym('x_h',6,1);   % hoist location
x_a=MX.sym('x_a',6,1);   % anchor location
m=MX.sym('m',1,1); % mass
g=MX.sym('off',3,1); % gravity
off = MX.sym('m',1,1); % mass
sys=MX.sym('sys',12,1); % Dummy var for nonlinear centroidal system definition

%% Auxillary equations
% Rotation matrices
Rx=[1 0 0; 
    0 cos(x(7)) -sin(x(7));
    0 sin(x(7)) cos(x(7))];
Ry=[cos(x(8)) 0 sin(x(8)); 
    0 1 0; 
    -sin(x(8)) 0 cos(x(8))];
Rz=[cos(x(9)) -sin(x(9)) 0; 
    sin(x(9)) cos(x(9)) 0; 
    0 0 1];

%Definition of the inertia matrix in the world frame
%TODO: CHANGE THIS PART OF SCRIPT THAT USES rpyToRot and Icom directly
w_R_b = Rz*Ry*Rx
% note Icom is defined as a row vector to be eval as an input like the
% others
WInertia=(w_R_b)*[Icom(1,1) Icom(2) Icom(3); Icom(4) Icom(5) Icom(6); Icom(7) Icom(8) Icom(9)]*(w_R_b');
inv_WInertia=inv(WInertia); % inverse of inertia matrix

%rope axes
CoM_pos = x(1:3);
hoist_pos1 = CoM_pos(:)+[0; -off;0.1];%left
hoist_pos2 =  CoM_pos(:)+[0; off;0.1];%right
fr1_hat = (hoist_pos1 - x_a(1:3))/ norm_2(hoist_pos1 - x_a(1:3))
fr2_hat = (hoist_pos2 - x_a(4:6)) / norm_2(hoist_pos2 - x_a(4:6))


%I_com^-1* (xfi-xc)*fr
%This is the product of I_com^-1 (omega_cross * I_com) in the First matrix
% This is a skew symmetric matrix for (xfi-xc) in the second matrix
%of equation 1.4 corressponding to omega_dot

%% Continous time system definition in terms of the symbolic variables based on state space equation 1.4



roll = x(7);
pitch = x(8);
yaw = x(9);


omega_x = x(10);
omega_y = x(11);
omega_z = x(12);
omega = x(10:12);

% Linear speed of COM
sys(1:3)=x(4:6);

% Linear acceleration of COM
sys(4:6)= g + (fr1_hat*u(1)+  fr2_hat*u(2))/m;

% Euler rates COM inv(E) * omega
euler_rates = [ cos(yaw)/cos(pitch)  (sin(yaw)/cos(pitch)) 0;
                -sin(yaw)   cos(yaw)                       0;
                cos(yaw)*tan(pitch)    sin(yaw)*tan(pitch) 1]*omega;

sys(7:9) = euler_rates;

% Angular acceleration of COM
sys(10:12)= -inv_WInertia*(cross_mx(x(10:12))*WInertia)*omega  + inv_WInertia*cross_mx(hoist_pos1-x(1:3)) *fr1_hat*u(1) +  inv_WInertia*cross_mx(hoist_pos2-x(1:3))*fr2_hat*u(2);

%Compute Jacobians
jacA=jacobian(sys,x);
jacB=jacobian(sys,u);


%% Jacobian functions based on equation 1.10
% defines what are the inputs and outputsof the sym function
Lin_A_func =Function('Lin_A',{[x;u;Icom; x_a;m;g; off]},{jacA});
Lin_B_func =Function('Lin_B',{[x;u;Icom; x_a;m;g; off]},{jacB});
% this is the offset  f(xbar,ubar)
%f_xubar_func = Function('f_xubar',{[x;u;Icom;xf;m;g;off]},{sys});

% Save jacobians and nonlinear function for offline use (Works only with casadi version>3.5.1)
% Lin_A.save('./casadiFunctions/Lin_A.casadi');
% Lin_B.save('./casadiFunctions/Lin_B.casadi');
% f_xubar.save('./casadiFunctions/f_xubar.casadi');

% Load the jacobians
% folder=fileparts(which('Lin_A.casadi'));
% Lin_A_func = Function.load(strcat(folder,'/Lin_A.casadi'));
% Lin_B_func = Function.load(strcat(folder,'/Lin_B.casadi'));
% f_xubar_func = Function.load(strcat(folder,'/f_xubar.casadi'));



%% Sensitivities evaluation
% equilibrium point and input parameters
const=constantsHyq();

% Robot mass
robotMass = 13;
base_width = 0.3; %X
base_length = 0.5; %Y
base_height = 0.3; %Z

hoist_offset = 0.3

% robot inertia
Ixx = 1/12*robotMass * (base_height^2 + base_length^2);
Iyy = 1/12*robotMass * (base_height^2 + base_width^2);
Izz = 1/12*robotMass * (base_length^2 + base_width^2);

robotInertia = diag([Ixx, Iyy, Izz]);
%robotInertia =  const.robotInertia;

% gravity vector
gravity = [0;0; -9.81];

for i =1:10
    CoM_pos = [5; 2.5; -9]; % it should not be in the middle otherwise if you se u1 = u2 you nullify the omega_dot!
    CoM_vel = [0; 0; 0]; %this does not make any difference

    psi = computeTheta(CoM_pos);
    %new model base frame (psi is associated to the rope plane so we have
    %a rotation of (pi/2 -psi) about the y axis
    % wRb =[ cos(pi/2-psi), 0, sin(pi/2-psi),
    %            0, 1,          0, 
    %         -sin(pi/2-psi), 0, cos(pi/2-psi)]


    rpy = [0.2;pi/2-psi;0.5];
    omega = [0.;0.0;0.0]; % this is needed to make show up in the A matrix the dependence on euler angles

    x_bar = [CoM_pos; % linearization point for the state
            CoM_vel;
            rpy;
            omega];
    u_bar = [-30; 25 - 5*i]

    anchor_pos = [0;0;0; 0; 5;0];



    nx=length(x_bar(:,1));
    %Evaluate the jacobians to give A and B matrices at linearization points and corresponding parameters
    A=Lin_A_func([x_bar;u_bar ;robotInertia(:);anchor_pos;robotMass;gravity; hoist_offset]);
    A=full(A);


    B=Lin_B_func([x_bar ;u_bar ;robotInertia(:);anchor_pos;robotMass;gravity; hoist_offset]);
    B=full(B);

    % eval stability of A around x_bar, u_bar
    [ V, D] = eigs(A);
    eig = eigs(A);
    stable = sum(abs(real(eig))< 1e-4);
    asstable = sum((real(eig)<0) & (abs(real(eig))> 1e-4));

    unstable = (6 - stable - asstable);
    fprintf("unstable: %d,  asstable: %d imag: %d\n", unstable, asstable,stable);

    %check var value
    % aaa =  euler_rates
    % aaa_fun = Function('aaa',{[x;u;Icom; x_a;m;g; off]},{aaa});
    % aaa_val = full(aaa_fun([x_bar;u_bar ;robotInertia(:);anchor_pos;robotMass;gravity; hoist_offset]))
    % 

    pause(1)

end










%Function evaluation at the linearization points
%[an1,an2,an3,an4,an5,an6,an7,an8,an9,an10,an11,an12]=f_xubar_func(x_bar(:,i),u_bar(:,i),robotInertia(:),footPos(:,i),robotMass,gravity);
%f_xbar_ubar=full([an1,an2,an3,an4,an5,an6,an7,an8,an9,an10,an11,an12])';
      
%Discretization with explict Euler eq 1.15 (we do no care about this
% Ts = 0.001;
% Lin_A_discrete = eye(nx) + Ts*A;
% Lin_B_discrete = Ts*B;

% Affine term in the linearized model eq 1.15
%r=Ts*f_xbar_ubar-Lin_A{i}*x_bar(:,i)-Lin_B{i}*u_bar(:,i);


