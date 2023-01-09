%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%slider model

clear all

syms l(t) theta(t) phi(t) sy(t) Fr Fut Fun Fs m ms g   real
ps = [0; sy; 0];
p = [l* sin(theta)*cos(phi); 
    sy + l* sin(theta)*sin(phi); 
    - l*cos(theta)]

ps_d = diff(ps, t);
p_d = diff(p, t);

v_2 = simplify(sum(p_d.^2),'Steps',50);
vs_2 = simplify(sum(ps_d.^2),'Steps',50);

Fr_v = [cos(phi)*sin(theta) sin(phi)*sin(theta) -cos(theta)];
Fut_v = [- sin(phi)  cos(phi) 0 ];
Fun_v = [cos(phi)*cos(theta) sin(phi)*cos(theta) sin(theta)];
Fs_v= [0 1 0];

L = 0.5*m*v_2 + 0.5*ms*vs_2 +  m*g*l*cos(theta)

% sy
dLdsy_dot = diff(L, diff(sy(t), t))
L1_1 = subs_args(diff(dLdsy_dot, t))
L1_2 = subs_args(diff(L, sy(t)))

dp_dsy = diff(p, sy(t))
dps_dsy = diff(ps, sy(t))
Q_1 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dsy + Fs*Fs_v*dps_dsy);




% theta 
dLdtheta_dot = diff(L, diff(theta(t), t))
L2_1 = subs_args(diff(dLdtheta_dot, t))
L2_2 = subs_args(diff(L, theta(t)))

dp_dtheta = diff(p, theta(t))
dps_dtheta = diff(ps, theta(t))
Q_2 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dtheta + Fs*Fs_v*dps_dtheta);





% phi 
dLdphi_dot = diff(L, diff(phi(t), t))
L3_1 = subs_args(diff(dLdphi_dot, t))
L3_2 = subs_args(diff(L, phi(t)))

dp_dphi = diff(p, phi(t))
dps_dphi = diff(ps, phi(t))
Q_3 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dphi + Fs*Fs_v*dps_dphi);




% l 
dLdl_dot = diff(L, diff(l(t), t))
L4_1 = subs_args(diff(dLdl_dot, t))
L4_2 = subs_args(diff(L, l(t)))

dp_dl = diff(p, l(t))
dps_dl = diff(ps, l(t))
Q_4 = subs_args((Fr*Fr_v + Fut*Fut_v + Fun*Fun_v)*dp_dl + Fs*Fs_v*dps_dl);

% equations

% simplify(L1_1 - L1_2) =  Q_1
% 
%  simplify(L2_1 - L2_2) =  Q_2
% 
% simplify(L3_1 - L3_2) =  Q_3
% 
% simplify(L4_1 - L4_2) =  Q_4

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
Lp = subs_args(L)

function out = subs_args(u)

out =subs(simplify(u),  {str2sym( 'diff(l(t), t)') , str2sym( 'diff(theta(t), t)'), str2sym('diff(phi(t), t)'), str2sym('diff(sy(t), t)')}, ...
                {str2sym('ld') , str2sym('thetad'),str2sym('phid'),str2sym('syd')});
            
out =subs(out,  {str2sym( 'diff(l(t), t, t)') , str2sym( 'diff(theta(t), t, t)'), str2sym('diff(phi(t), t, t)'), str2sym('diff(sy(t), t, t)')}, ...
                {str2sym('ldd') , str2sym('thetadd'),str2sym('phidd'),str2sym('sydd')});
            
out =subs(out, {str2sym('l(t)') , str2sym('theta(t)'),str2sym('phi(t)'),str2sym('sy(t)')},... 
                {str2sym('l'), str2sym('theta'),str2sym('phi'),str2sym('sy')});
        
end