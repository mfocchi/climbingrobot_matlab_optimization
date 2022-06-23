
%l const
syms l theta(t) phi(t) t g f1 f2 g1 g2 g3


% compute jacobian symblically
p = [l* sin(theta)*cos(phi); 
    l* sin(theta)*sin(phi); 
    - l*cos(theta)]
pd = diff(p, t);
pd =subs( pd,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('thetad'),str2sym('phid')});
pd =subs( pd, {str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('theta'),str2sym('phi')});


theta = theta(t)
phi = phi(t)
thetad=diff(theta,t)
phid = diff(phi,t)
f =  cos(theta)*sin(theta)*(phid^2)-(g/l)*sin(theta);
g = -2*(cos(theta)/sin(theta))*phid*thetad;

df_dtheta = diff(f, theta)
df_dthetad = diff(f, thetad)
df_dphi = diff(f, phi)
df_dphid = diff(f, phid)


dg_dtheta = diff(g, theta)
dg_dthetad = diff(g, thetad)
dg_dphi = diff(g, phi)
dg_dphid = diff(g, phid)


A = [0                1              0    0 ;
     df_dtheta ,  df_dthetad , df_dphi, df_dphid ;
    0                0                    0            1 ;
     dg_dtheta ,  dg_dthetad , dg_dphi, dg_dphid ]

A =subs( A,  {str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)')}, {str2sym('thetad'),str2sym('phid')});
A =subs( A, {str2sym('theta(t)'),str2sym('phi(t)')}, {str2sym('theta'),str2sym('phi')});


As = [0 1 0 0 ;
      f1  0 0 f2;
        0 0 0 1;
        g1 g2 0 g3]

D= simplify(eig(As))
simplify(imag(D(4)))

% l non const 






% compute target for mutiple jumps symbolically
syms th1 th2 th3  l1p l2p l3p
% rotation about X axis
Rx=@(angle) [1       0        0
            0       cos(angle) -sin(angle)
            0       sin(angle)  cos(angle)]
    

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

% comupute l1 l2 l3 with law of cosines
% l1 = l1p
% l2 = l1^2+l2p^2 -2*l1*l2p*cos(pi - th1)
% l3 = l2^2+l3p^2 -2*l2*l3p*cos(pi - th2)

