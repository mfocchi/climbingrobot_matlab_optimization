function w_R_b = computeOrientation(p_base, p_anchor1, p_anchor2)


l1 = norm(p_base - p_anchor1);
l2 = norm(p_base - p_anchor2);
psi = computePsi(p_base);
b = norm(p_anchor2 - p_anchor1);

% 
% syms psi  l1 l2 b alpha   real 
% 
% % for some reason acos() returns pi -acos so I need to subtract from pi to
% % get acos
% alpha = (pi - acos((l2^2 -b^2 -l1^2)/(2*b*l1)))
% 
% 
% Rx=@(angle) [1       0        0
%             0       cos(angle) -sin(angle)
%             0       sin(angle)  cos(angle)];
%     
% 
% Ry=@(angle)[cos(angle)  0      sin(angle)
%             0           1      0
%          -sin(angle) 0      cos(angle)];
% 
% Rz=@(angle)[cos(angle)    -sin(angle) 0
%             sin(angle)   cos(angle)   0
%             0           0             1];
% %%%%%%%%%%%%%%%%%%%%%%%
% % WF is assumed in first anchor the left one
% % the state variables are: psi, alpha, l1
% % 1) rotate pi/2-psi about the y axis
% H0_T_1 = [Ry(pi/2-psi) , [0;0;0]
%           zeros(1, 3), 1]      
% 
% % 2) rotate pi/2-alpha about z' axis
% H1_T_2 = [Rz(pi/2-alpha) , [0;0;0]
%           zeros(1, 3), 1]  
%  
% % 3) move l1 along x'' axis
% H2_T_3 = [ eye(3), [l1;0;0]
%           zeros(1, 3), 1]    
% % thes last ywo trotations do no affect the position and are thee to have consistency in orientation with the gazebo model       
% % 4) rotate  -(pi/2-alpha) back Z''' axis to have base Y axis aligned with
% % anchors
% H3_T_4 = [Rz(-(pi/2-alpha)) , [0;0;0]
%           zeros(1, 3), 1] 
% 
% % 4) rotate  -(pi/2-alpha) back Z''' axis to have base Y axis aligned with
% % anchors      
% H4_T_b = [    0    0   -1    0
%               0    1    0    0
%               1    0    0    0
%               0    0    0    1 ];    
%  % 5) rotate   back -pi/2 about Y'''' axis to have base X asis sabe as in
%  % gazebo
% H0_T_b =  simplify(H0_T_1 *H1_T_2*H2_T_3 * H3_T_4*H4_T_b);



w_R_b=[            cos(psi), 0, -sin(psi),
                        0, 1,        0,                                 
                      sin(psi), 0,  cos(psi)];

end