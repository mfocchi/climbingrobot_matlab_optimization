clear all
%l const
syms l(t) theta(t) phi(t) t g f1 f2 g1 g2 g3



theta = theta(t)
phi = phi(t)
thetad=diff(theta,t)
phid = diff(phi,t)
l = l(t)
ld = diff(l,t)

% ld = 0 
f1 =  cos(theta)*sin(theta)*(phid^2)-(g/l)*sin(theta); % thetadd
f2 = -2*(cos(theta)/sin(theta))*phid*thetad; % phidd

df1_dtheta = diff(f1, theta)
df1_dthetad = diff(f1, thetad)
df1_dphi = diff(f1, phi)
df1_dphid = diff(f1, phid)

df2_dtheta = diff(f2, theta)
df2_dthetad = diff(f2, thetad)
df2_dphi = diff(f2, phi)
df2_dphid = diff(f2, phid)

% to linearize we need to do derivatives of f(x),g(x) and put in state space form 
% x = [theta  thetad phi phid]

A = [0                1              0    0 ;
     df1_dtheta ,  df1_dthetad , df1_dphi, df1_dphid ;
    0                0                    0            1 ;
     df2_dtheta ,  df2_dthetad , df2_dphi, df2_dphid ]

A =subs( A,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('thetad'),str2sym('phid')});
A =subs( A, {str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('theta'),str2sym('phi')});


% by hand
% As = [0 1 0 0 ;
%       a  0 0 b;
%         0 0 0 1;
%         c  d  e  f]
% 
% D= simplify(eig(As))
% simplify(imag(D(4)))

%small oscillations 
A = [ 0    1     0     0;
     phid^2 + g/l  0   0    0;
     0    0    0     1;
     2*phid*thetad/theta^2 -2*phid/theta  0   -2*thetad/theta];
A =subs( A,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('thetad'),str2sym('phid')});
A =subs( A, {str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('theta'),str2sym('phi')});

% small oscill simple pendulum
A=[ 0 1; -g/l 0] 
%T = 2*pi*sqrt(l/g)

% big oscill lin pendulum
A=[ 0 1; -g/l*cos(theta)  0] 
%T = 2*pi*sqrt(l/(g*cos(theta)))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% l non const 
f =  cos(theta)*sin(theta)*(phid^2)  - 2/l* thetad*ld -(g/l)*sin(theta); % thetadd
g = -2*(cos(theta)/sin(theta))*phid*thetad - 2/l*phid*ld; % phidd
h = l*thetad^2 + l*sin(theta)^2*phid^2 + g*cos(theta);

df_dtheta = diff(f, theta)
df_dthetad = diff(f, thetad)
df_dphi = diff(f, phi)
df_dphid = diff(f, phid)
df_dl = diff(f, l)
df_dld = diff(f, ld)

df2_dtheta = diff(g, theta)
df2_dthetad = diff(g, thetad)
dg_dphi = diff(g, phi)
dg_dphid = diff(g, phid)
dg_dl = diff(g, l)
dg_dld = diff(g, ld)

dh_dtheta = diff(h, theta)
dh_dthetad = diff(h, thetad)
dh_dphi = diff(h, phi)
dh_dphid = diff(h, phid)
dh_dl = diff(h, l)
dh_dld = diff(h, ld)

A = [0                1              0               0            0                  0;
     df_dtheta ,  df_dthetad ,       0,       df_dphid,           df_dl,       df_dld  ;
     0                0              0               1            0                  0;
     df2_dtheta ,  df2_dthetad , dg_dphi, dg_dphid     ,          dg_dl,        dg_dld  ;                      
     0                0              0               0            0                  1;
      dh_dtheta ,  dh_dthetad , dh_dphi, dh_dphid     ,          dh_dl,        dh_dld  ];
A =subs( A,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)'), str2sym('diff(l(t), t)')}, {str2sym('thetad'),str2sym('phid'),str2sym('ld')});
A =subs( A, {str2sym('theta(t)'),str2sym('phi(t)'),str2sym('l(t)')}, {str2sym('theta'),str2sym('phi'),str2sym('l')}); 
  
A_= simplify(A);

% check gain 
fun = matlabFunction(A) % g,l,ld,phid,theta,thetad
theta0 = 0.1
for i =1: 10 
x1(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[1; 0; 0; 0;0;0])
x2(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[0; 1; 0; 0;0;0])
x3(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[0; 0; 1; 0;0;0])
x4(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[0; 0; 0; 1;0;0])
x5(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[0; 0; 0; 0;1;0])
x6(i) = norm(fun(9.81, 3, 0,0, theta0, 0)*[0; 0; 0; 0;0;1])














%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute COM velocity  symblically for pathlength computation
syms l(t) theta(t) phi(t)
p = [l* sin(theta)*cos(phi); 
    l* sin(theta)*sin(phi); 
    - l*cos(theta)]
pd = diff(p, t);
pd =subs( pd,  {str2sym( 'diff(l(t), t)') , str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('ld') , str2sym('thetad'),str2sym('phid')});
pd =subs( pd, {str2sym('l(t)') , str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('l'), str2sym('theta'),str2sym('phi')});
pd


% compute kinematic simbolically for the simplifie model
syms phi theta l  
Rx=@(angle) [1       0        0
            0       cos(angle) -sin(angle)
            0       sin(angle)  cos(angle)]
    

Ry=@(angle)[cos(angle)  0      sin(angle)
            0           1      0
         -sin(angle) 0      cos(angle)]

Rz=@(angle)[cos(angle)    -sin(angle) 0
            sin(angle)   cos(angle)   0
            0           0             1]

H0_T_1 = [Rz(phi) , [0;0;0]
          zeros(1, 3), 1]

H1_T_2 = [Ry(-theta) , [0;0;0]
          zeros(1, 3), 1]      
      
H1_T_2 = [Ry(pi/2-theta) , [0;0;0]
          zeros(1, 3), 1]      
   
H2_T_b = [ eye(3), [l;0;0]
          zeros(1, 3), 1]      
      
H0_T_b =  simplify(H0_T_1 *H1_T_2*H2_T_b)     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
% computeJointVariables from cartesian in Gazebo
% in gazebo you first rotate pitch about -Y then roll about X then you
% translate rope along -Z

syms pitch roll rope real
Rsym_x = @(angle)[	1   ,    0     	  ,  	  0,
                0   ,    cos(angle) ,  -sin(angle),
                0   ,    sin(angle) ,  cos(angle)];
Rsym_y = @(angle)[cos(angle) ,	 0  ,   sin(angle),
          0       ,    1  ,   0,
          -sin(angle) 	,	 0  ,  cos(angle)];

Rsym_z = @(angle)[cos(angle), -sin(angle), 0,
                 sin(angle), cos(angle), 0,
                    0, 0, 1];

% the rotation about -Y is given by the transposed
H0_T_1 = [Rsym_y(pitch)', [0;0;0]
          zeros(1, 3), 1];

H1_T_2 = [Rsym_x(roll) , [0;0;0]
          zeros(1, 3), 1] ;     
      
H2_T_3 = [eye(3) , [0;0;-rope]
          zeros(1, 3), 1];            
      
H0_T_3 =  simplify(H0_T_1 *H1_T_2*H2_T_3)     

p = H0_T_3(1:3, 4)
joint_l = simplify(norm(p))
joint_pitch = simplify(atan2(p(1), -p(3)))
joint_roll = simplify(asin(p(2)/joint_l))












%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute target for mutiple jumps symbolically
    


% we need to use hom transofrms wrt to FIXED axes
% translate l1 along -Z
H0_T_1 = [eye(3) , [0;0;-l1p]
          zeros(1, 3), 1]

%rotate th1 around X      
H1_T_2 = [Rx(th1), [0;0;0]
          zeros(1, 3), 1]

% translate l2p  along -Z
H2_T_3 = [eye(3) , [0;0;-l2p]
          zeros(1, 3), 1]
    
% rotate th2  around  X
H3_T_4 = [Rx(th2), [0;0;0]
          zeros(1, 3), 1]
          

% translate l3p  along -Z
H4_T_5 = [eye(3) , [0;0;-l3p]
          zeros(1, 3), 1]    
      
% rotate th3  around  X
H5_T_6 = [Rx(th3), [0;0;0]
          zeros(1, 3), 1]

% for fixed axes yoiu need to reverse order  
H0_T_2 =  H1_T_2*H0_T_1;
H0_T_4 =  H3_T_4 *H2_T_3*H1_T_2*H0_T_1;
H0_T_6 = H5_T_6 *  H4_T_5 * H3_T_4 *H2_T_3*H1_T_2*H0_T_1;

target_1 = simplify(H0_T_2(1:3,4))
target_2 = simplify(H0_T_4(1:3,4))
target_3 = simplify(H0_T_6(1:3,4))

%sanity check (thtetavec = [pi/2 pi/4  pi/4], l = [1,1,sqrt(2)] 
H0_Tcheck = subs(H0_T_6,{'th1', 'th2', 'th3','l1p', 'l2p','l3p'},{pi/2,pi/4,pi/4,1,1,sqrt(2)}) 
eval(H0_Tcheck(1:3,1:3)) -Rx(pi/2+pi/4+pi/4)
eval(H0_Tcheck(1:3,4)) -[0;2;0]


