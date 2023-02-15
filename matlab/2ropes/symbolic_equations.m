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
% 1) rotate pi/2-psi about the y axis
H0_T_1 = [Ry(pi/2-psi) , [0;0;0]
          zeros(1, 3), 1]      

% 2) rotate alpha about z axis
H1_T_2 = [Rz(alpha) , [0;0;0]
          zeros(1, 3), 1]  
 
% 3) move l1 along x
H2_T_b = [ eye(3), [l1;0;0]
          zeros(1, 3), 1]      
      
H0_T_b =  simplify(H0_T_1 *H1_T_2*H2_T_b)  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get a simplified model
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




% Compute Lagrangian equations
%
% v_2 = simplify(sum(p_d.^2),'Steps',50);
% Fr1_v = ...
% Fr2_v = ...
% L = 0.5*m*v_2 +  m*g*l*cos(psi)
% 
% 
% %GDL: l1 
% dLdl1_dot = diff(L, diff(l1(t), t))
% L1_1 = subs_args(diff(dLdl1_dot, t))
% L1_2 = subs_args(diff(L, l1(t)))
% 
% Q_1 = ...

% % GDL: l2 
% dLdl2_dot = diff(L, diff(l2(t), t))
% L2_1 = subs_args(diff(dLdl2_dot, t))
% L2_2 = subs_args(diff(L, l2(t)))
% 
% Q_2 = ...
% 
% % GDL: psi 
% dLdl_dot = diff(L, diff(psi(t), t))
% L3_1 = subs_args(diff(dLdpsi_dot, t))
% L3_2 = subs_args(diff(L, psi(t)))
% 
% Q_3 = ...

% equations
% simplify(L1_1 - L1_2) =  Q_1
% 
%  simplify(L2_1 - L2_2) =  Q_2
% 
% simplify(L3_1 - L3_2) =  Q_3
% 

function out = subs_args(u)

out =subs(u, {str2sym('l1(t)') , str2sym('l2(t)'),str2sym('psi(t)')},   {str2sym('l1'), str2sym('l2'),str2sym('psi')});
                          
out =subs(out,  {str2sym( 'diff(l1(t), t)') , str2sym( 'diff(l2(t), t)'), str2sym('diff(psi(t), t)')},   {str2sym('l1d') , str2sym('l2d'),str2sym('psid') });
                        
out =subs(out,  {str2sym( 'diff(l1(t), t, t)') , str2sym( 'diff(l2(t), t, t)'), str2sym('diff(psi(t), t, t)')},  {str2sym('l1dd') , str2sym('l2dd'),str2sym('psidd')});


end




