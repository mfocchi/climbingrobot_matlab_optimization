% compute kinematic simbolically for the 2 rope model minimum
% representation

syms psi  l1 l2 b alpha

alpha = acos((l2^2 -b^2 -l1^2)/(2*b*l1));



Rx=@(angle) [1       0        0
            0       cos(angle) -sin(angle)
            0       sin(angle)  cos(angle)];
    

Ry=@(angle)[cos(angle)  0      sin(angle)
            0           1      0
         -sin(angle) 0      cos(angle)];

Rz=@(angle)[cos(angle)    -sin(angle) 0
            sin(angle)   cos(angle)   0
            0           0             1];
%%%%%%%%%%%%%%%%%%%%%%%
% 1) rotate pi-psi about the y axis
H0_T_1 = [Ry(pi/2-psi) , [0;0;0]
          zeros(1, 3), 1]      

% 2) rotate alpha about z axis
H1_T_2 = [Rz(alpha) , [0;0;0]
          zeros(1, 3), 1]  
 
% move l1 along x
H2_T_b = [ eye(3), [l1;0;0]
          zeros(1, 3), 1]      
      
H0_T_b =  simplify(H0_T_1 *H1_T_2*H2_T_b)  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
syms l1(t) l2(t) psi(t) b Fr1 Fr2 Fleg  real

p = [-(sin(psi)*(b^2 + l1^2 - l2^2))/(2*b);
     l1*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);
     (cos(psi)*(b^2 + l1^2 - l2^2))/(2*b)]

p_d = simplify(diff(p, t),'Steps',50);
p_dd =  diff(p_d, t);

p_dd1 =subs(p_dd,  {str2sym( 'diff(l1(t), t, t)') , str2sym( 'diff(l2(t), t, t)'), str2sym('diff(psi(t), t, t)')},  {str2sym('l1dd') , str2sym('l2dd'),str2sym('psidd')});                       
p_dd2 =subs(p_dd1,  {str2sym( 'diff(l1(t), t)') , str2sym( 'diff(l2(t), t)'), str2sym('diff(psi(t), t)')},   {str2sym('l1d') , str2sym('l2d'),str2sym('psid') });                       
p_dd3 =subs(p_dd2, {str2sym('l1(t)') , str2sym('l2(t)'),str2sym('psi(t)')},   {str2sym('l1'), str2sym('l2'),str2sym('psi')});




% 
% v_2 = simplify(sum(p_d.^2),'Steps',50);
% Fr1_v = [cos(phi)*sin(theta) sin(phi)*sin(theta) -cos(theta)];
% Fr2_v = [- sin(phi)  cos(phi) 0 ];
% L = 0.5*m*v_2 +  m*g*l*cos(psi)
% 
% 
% % l1 
% dLdtheta_dot = diff(L, diff(theta(t), t))
% L2_1 = subs_args(diff(dLdtheta_dot, t))
% L2_2 = subs_args(diff(L, theta(t)))
% 
% dp_dtheta = diff(p, theta(t))
% dps_dtheta = diff(ps, theta(t))
% Q_2 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dtheta + Fs*Fs_v*dps_dtheta);
% 
% % l2 
% dLdphi_dot = diff(L, diff(phi(t), t))
% L3_1 = subs_args(diff(dLdphi_dot, t))
% L3_2 = subs_args(diff(L, phi(t)))
% 
% dp_dphi = diff(p, phi(t))
% dps_dphi = diff(ps, phi(t))
% Q_3 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dphi + Fs*Fs_v*dps_dphi);
% 
% % psi 
% dLdl_dot = diff(L, diff(l(t), t))
% L4_1 = subs_args(diff(dLdl_dot, t))
% L4_2 = subs_args(diff(L, l(t)))
% 
% dp_dl = diff(p, l(t))
% dps_dl = diff(ps, l(t))
% Q_4 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dl + Fs*Fs_v*dps_dl);

% equations

% simplify(L1_1 - L1_2) =  Q_1
% 
%  simplify(L2_1 - L2_2) =  Q_2
% 
% simplify(L3_1 - L3_2) =  Q_3
% 
% simplify(L4_1 - L4_2) =  Q_4

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
%Lp = subs_args(L)

function out = subs_args(u)

out =subs(u, {str2sym('l1(t)') , str2sym('l2(t)'),str2sym('psi(t)')},   {str2sym('l1'), str2sym('l2'),str2sym('psi')});
                          
out =subs(out,  {str2sym( 'diff(l1(t), t)') , str2sym( 'diff(l2(t), t)'), str2sym('diff(psi(t), t)')},   {str2sym('l1d') , str2sym('l2d'),str2sym('psid') });
                        
out =subs(out,  {str2sym( 'diff(l1(t), t, t)') , str2sym( 'diff(l2(t), t, t)'), str2sym('diff(psi(t), t, t)')},  {str2sym('l1dd') , str2sym('l2dd'),str2sym('psidd')});


end




