
% Linearization function for nonlinear centroidal dynamics model x_dot = f(x,u,footPos)

%% Symbolic variables in Casadi
import casadi.*
x = MX.sym('x',12,1) ;  % States
u = MX.sym('u',2,1);   % Inputs
Icom=MX.sym('Icom',9,1); % Inertia
x_h=MX.sym('x_h',6,1);   % hoist location
x_a=MX.sym('x_a',6,1);   % anchor location
m=MX.sym('m',1,1); % mass
g=MX.sym('g',3,1); % gravity
sys=MX.sym('sys',12,1); % Dummy var for nonlinear centroidal system definition

%% Auxillary equations
% Rotation matrices
Rx=[1 0 0; 0 cos(x(7)) sin(x(7)); 0 -sin(x(7)) cos(x(7))];
Ry=[cos(x(8)) 0 -sin(x(8)); 0 1 0; sin(x(8)) 0 cos(x(8))];
Rz=[cos(x(9)) sin(x(9)) 0; -sin(x(9)) cos(x(9)) 0; 0 0 1];

%Definition of the inertia matrix in the world frame
%TODO: CHANGE THIS PART OF SCRIPT THAT USES rpyToRot and Icom directly
WInertia=(Rx*Ry*Rz)'*[Icom(1) Icom(2) Icom(3); Icom(4) Icom(5) Icom(6); Icom(7) Icom(8) Icom(9)]*(Rx*Ry*Rz);
inv_WInertia=inv(WInertia); % inverse of inertia matrix

%rope axes
fr1_hat = (x_h(1:3) -x_a(1:3))/ norm_2(x_h(1:3) - x_a(1:3))
fr2_hat = (x_h(4:6)- x_a(4:6)) / norm_2(x_h(4:6) - x_a(4:6))


%I_com^-1* (xfi-xc)*fr
%This is the product of I_com^-1 (omega_cross * I_com) in the First matrix
% This is a skew symmetric matrix for (xfi-xc) in the second matrix
%of equation 1.4 corressponding to omega_dot

%% Continous time system definition in terms of the symbolic variables based on state space equation 1.4

% Linear speed of COM
sys(1:3)=x(4:6);

% Linear acceleration of COM
sys(4:6)= g + (fr1_hat*u(1)+  fr2_hat*u(2))/m;

% Angular speed of COM
sys(7)=(cos(x(9))/cos(x(8)))*x(10)+ (sin(x(9))/cos(x(8)))*x(11);
sys(8)=-sin(x(9))*x(10)+ cos(x(9))*x(11);
sys(9)=cos(x(9))*tan(x(8))*x(10)+sin(x(9))*tan(x(8))*x(11)+x(12);

% Angular acceleration of COM
sys(10:12)= -inv_WInertia*(cross_mx(x(10:12))*WInertia)*x(10:12)  + inv_WInertia*cross_mx(x_h(1:3)-x(1:3))*fr1_hat*u(1) +  inv_WInertia*cross_mx(x_h(4:6)-x(1:3))*fr2_hat*u(2);

%Compute Jacobians
jacA=jacobian(sys,x);
jacB=jacobian(sys,u);

%% Jacobian functions based on equation 1.10
% defines what are the inputs and outputsof the sym function
Lin_A_func =Function('Lin_A',{[x;u;Icom;x_h; x_a;m;g]},{jacA});
Lin_B_func =Function('Lin_B',{[x;u;Icom;x_h; x_a;m;g]},{jacB});
% this is the offset  f(xbar,ubar)
%f_xubar_func = Function('f_xubar',{[x;u;Icom;xf;m;g]},{sys});

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
% Load constants for HyQ
const=constantsHyq();
% Robot mass
robotMass = const.robotMass;
% robot inertia
robotInertia = const.robotInertia;

% robotMass = 5;
% robotInertia = eye(3)*2;

% gravity vector
gravity = const.gravity;

CoM_pos = [1; 2.5; -9];
CoM_vel = [0; 0; 0];

psi = computeTheta(CoM_pos);
%new model base frame (psi is associated to the rope plane so we have
%a rotation of (pi/2 -psi) about the y axis
% wRb =[ cos(pi/2-psi), 0, sin(pi/2-psi),
%            0, 1,          0, 
%         -sin(pi/2-psi), 0, cos(pi/2-psi)]


rpy = [0;pi/2-psi;0];
omega = [0;0;0];

x_bar = [CoM_pos; % linearization point for the state
        CoM_vel;
        rpy;
        omega];
u_bar = [-20; -10];

anchor_pos = [0;0;0; 0; 10;0];
hoist_pos = [CoM_pos+[0; -0.1;0];
              CoM_pos+[0; 0.1;0]];


nx=length(x_bar(:,1));
%Evaluate the jacobians to give A and B matrices at linearization points and corresponding parameters
A=Lin_A_func([x_bar;u_bar ;robotInertia(:);hoist_pos;anchor_pos;robotMass;gravity]);
A=full(A)
        
B=Lin_B_func([x_bar ;u_bar ;robotInertia(:);hoist_pos;anchor_pos;robotMass;gravity]);
B=full(B);
        
%Function evaluation at the linearization points
%[an1,an2,an3,an4,an5,an6,an7,an8,an9,an10,an11,an12]=f_xubar_func(x_bar(:,i),u_bar(:,i),robotInertia(:),footPos(:,i),robotMass,gravity);
%f_xbar_ubar=full([an1,an2,an3,an4,an5,an6,an7,an8,an9,an10,an11,an12])';

        
%Discretization with explict Euler eq 1.15
Ts = 0.001;
Lin_A_discrete = eye(nx) + Ts*A;
Lin_B_discrete = Ts*B;

% Affine term in the linearized model eq 1.15
%r=Ts*f_xbar_ubar-Lin_A{i}*x_bar(:,i)-Lin_B{i}*u_bar(:,i);

% eval stability of A around x_bar, u_bar
[ V, D] = eigs(A)
eigs(A)
