% compute kinematic simbolically for the 2 rope model minimum
% representation
clear all 
syms psi  l1 l2 b alpha   real 

% for some reason acos() returns pi -acos so I need to subtract from pi to
% get acos
alpha = (pi - acos((l2^2 -b^2 -l1^2)/(2*b*l1)))


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
% WF is assumed in first anchor the left one
% the state variables are: psi, alpha, l1
% 1) rotate pi/2-psi about the y axis
H0_T_1 = [Ry(pi/2-psi) , [0;0;0]
          zeros(1, 3), 1]      

% 2) rotate pi/2-alpha about z' axis
H1_T_2 = [Rz(pi/2-alpha) , [0;0;0]
          zeros(1, 3), 1]  
 
% 3) move l1 along x'' axis
H2_T_b = [ eye(3), [l1;0;0]
          zeros(1, 3), 1]      
      
H0_T_b =  simplify(H0_T_1 *H1_T_2*H2_T_b)  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get a simplified model (states l1, l2, psi)
clear all
syms l1(t) l2(t) psi(t) b Fr1 Fr2 Fleg  real

p = [l1*sin(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2);
      (b^2 + l1^2 - l2^2)/(2*b);
     -l1*cos(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2)]
alpha = (pi - acos((l2^2 -b^2 -l1^2)/(2*b*l1)))

%check value is correct should be p = [0, 2.5, -6 ] for 0, 6.5, 6.5
p_val = vpa(subs(p,{psi(t),l1(t),l2(t), b},{0, 6.5, 6.5,5}),4)
alpha_val = vpa(subs(alpha, {psi(t),l1(t),l2(t), b},{0, 6.5, 6.5,5}),4)
 
% clean
p1 = subs(p, {str2sym('l1(t)') , str2sym('l2(t)'),str2sym('psi(t)')},   {str2sym('l1'), str2sym('l2'),str2sym('psi')});


p_d = simplify(diff(p, t),'Steps',50);
p_dd =  diff(p_d, t);

p_dd1 =subs(p_dd,  {str2sym( 'diff(l1(t), t, t)') , str2sym( 'diff(l2(t), t, t)'), str2sym('diff(psi(t), t, t)')},  {str2sym('l1dd') , str2sym('l2dd'),str2sym('psidd')});                       
p_dd2 =subs(p_dd1,  {str2sym( 'diff(l1(t), t)') , str2sym( 'diff(l2(t), t)'), str2sym('diff(psi(t), t)')},   {str2sym('l1d') , str2sym('l2d'),str2sym('psid') });                       
p_dd3 =subs(p_dd2, {str2sym('l1(t)') , str2sym('l2(t)'),str2sym('psi(t)')},   {str2sym('l1'), str2sym('l2'),str2sym('psi')});


% trick to replace the sqrt
p_dd4 = simplify(p_dd3)
p_dd5 =subs(p_dd4, {str2sym('sin(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2)')} ,  {str2sym('px_l1') });
p_dd6 =subs(p_dd5, {str2sym('cos(psi)*(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2)')} ,  {str2sym('n_pz_l1') });
p_dd7 =subs(p_dd6, {str2sym('(1 - (b^2 + l1^2 - l2^2)^2/(4*b^2*l1^2))^(1/2)')} ,  {str2sym('px_l1_sinpsi') });
p_dd8 =subs(p_dd7, {str2sym(' (b^2 + l1^2 - l2^2)')} ,  {str2sym('py2b') });


syms px_l1  n_pz_l1 px_l1_sinpsi py2b            psidd l1dd l2dd A b real
select_1 = [1 0 0]
select_2 = [0 1 0]
select_3 = [0 0 1]
A = sym(zeros(3,3));
b = sym(zeros(3,1));
coeff_psidd = coeffs(select_1*p_dd8, psidd) 
A(1,1) = coeff_psidd*[0;1];
coeff_l1dd = coeffs(coeff_psidd*[1;0], l1dd) 
A(1,2) = coeff_l1dd*[0;1];
coeff_l2dd = coeffs(coeff_l1dd*[1;0], l2dd) 
A(1,3) = coeff_l2dd*[0;1]
b(1,1) = coeff_l2dd*[1;0]

coeff_psidd2 = coeffs(select_2*p_dd8, psidd)  % is only one element cause this line does not contain psidd
A(2,1) = 0;
coeff_l1dd2 = coeffs(coeff_psidd2, l1dd) 
A(2,2) = coeff_l1dd2*[0;1];
coeff_l2dd2 = coeffs(coeff_l1dd2*[1;0], l2dd) 
A(2,3) = coeff_l2dd2*[0;1]
b(2,1) = coeff_l2dd2*[1;0]

coeff_psidd = coeffs(select_3*p_dd8, psidd) 
A(3,1) = coeff_psidd*[0;1];
coeff_l1dd = coeffs(coeff_psidd*[1;0], l1dd) 
A(3,2) = coeff_l1dd*[0;1];
coeff_l2dd = coeffs(coeff_l1dd*[1;0], l2dd) 
A(3,3) = coeff_l2dd*[0;1]
b(3,1) = coeff_l2dd*[1;0]

b = simplify(b,'Steps',50)
% check everything is allright
simplify( (A*[psidd; l1dd; l2dd] + b) - p_dd8)




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




