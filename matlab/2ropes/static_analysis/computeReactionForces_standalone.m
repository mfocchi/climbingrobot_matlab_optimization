
close all 
clear all
clc

anchor_distance = 5;
p_anchor1 = [0;0;0];
p_anchor2 = [0;anchor_distance;0];

% x = [fl1, fl2, fr1, fr2]
baseline = 0.8;
wall_clearance = 0.4;


% IMPORTANT MAX ROPE FORCE OR FEET FORCE IS NOT CONSIDERED!
mu = 0.8;
g_vec = [0;0;-9.81];
mass = 10;
force_scale =0.1;
min_feet_force = 0;
% margin out of the vertical
margin = 0.31;
 
figure;

% base pos
p_base = [wall_clearance; 1.5; -3];
   
wRb = computeOrientation(p_base, p_anchor1, p_anchor2);
    
%feet pos in BF
pf1 = [-wall_clearance; -baseline/2; 0];
pf2 = [-wall_clearance; baseline/2; 0];

%hoist positions in BF
ph1 = [0; -0.2; 0.1];
ph2 = [0; 0.2; 0.1];


% feet position in WF
w_pf1 = wRb*pf1 +p_base;
w_pf2 = wRb*pf2 +p_base; 

% rope attachment points in WF
w_ph1 = wRb*ph1 +p_base;
w_ph2 = wRb*ph2 +p_base; 

% line of actions of the anchor forces (univ vectors)
w_pr1  = (w_ph1-p_anchor1)/norm(w_ph1-p_anchor1);
w_pr2  = (w_ph2-p_anchor2)/norm(w_ph2-p_anchor2);

% optim var are fl1(3x1), fl2(3x1), fr1 (scalar), fr2(scalar)
% #∥Px−p∥2# 
% # G =  Mt*M
% # g = -MT*b
% #0.5 xT*G*x + g*x
% #s.t. Cx≤d
% #     Ax=b

w_feet  = 0.1;
w_rope = 0.01;
M = blkdiag(w_feet*eye(3),w_feet*eye(3),w_rope,w_rope);
m = zeros(3+3+1+1,1);
G = 2*M'* M;
g = -2*m'* M;

% C = [1., 2., 1.; 2., 0., 1.; -1., 2., -1.];
% d = [3.; 2.; -2.];

% force equilibrium  constraints (3)
Aeq1 = [eye(3) eye(3) w_pr1 w_pr2]; 
beq1 = -mass*g_vec;

% moment equilibrium  constraints about base origin(3)
Aeq2 = [cross_mx(w_pf1 - p_base) cross_mx(w_pf2 - p_base) cross_mx(w_ph1 - p_base)*w_pr1  cross_mx(w_ph2 - p_base)*w_pr2] ;
beq2 = zeros(3,1);

% stack equality constraints
Aeq = [Aeq1;Aeq2];
beq = [beq1;beq2];

%friction cone matrix
%contact frame
n = [1;0;0];   
t1 = cross(n, [0;1;0]);
t2 = cross( [0;0;1], n);

%friction cone pyramid
F = [(t1-mu*n)';
     (-t1-mu*n)';
     (t2-mu*n)';
     (-t2-mu*n)'];

% inequalities friction cones constraints + min normal force + 
% unilaterality of ropes  fr1 < 0 
% forces
C = blkdiag([F ;-n'],[F; -n'], 1,1);
d = [zeros(4,1);-min_feet_force; zeros(4,1);-min_feet_force;0;0];

% only unilateral on feet and rope used for optim
%     C = blkdiag(-n',-n', 1,1);
%     d = [-min_feet_force; -min_feet_force;0;0]

[x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, C, d, Aeq, beq);
%[x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, [], [], Aeq, beq);
fl1 = x(1:3);
fl2 = x(4:6);
fr1 = x(7)
fr2 = x(8)



 

min_z = -7;
max_z = 1;
min_y = p_anchor1(2)-2;
max_y = p_anchor2(2)+2;

% for paper 
%clf(gcf)
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultTextFontSize', 30);
set(0, 'DefaultAxesFontSize', 30);
set(0, 'DefaultUicontrolFontSize', 30);

%     drawing a wall at X = 0
p1 = [0 min_y min_z];
p2 = [0 max_y min_z];
p3 = [0 max_y max_z];
p4 = [0 min_y max_z];
Xw = [p1(1) p2(1) p3(1) p4(1)];
Yw = [p1(2) p2(2) p3(2) p4(2)];
Zw = [p1(3) p2(3) p3(3) p4(3)];
h = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  );
%draw feasible area
fill3([0.005,0.005,0.005,0.005] , [p_anchor1(2)-margin,p_anchor1(2)-margin , p_anchor2(2)+margin,  p_anchor2(2)+margin], [-7,0, 0, -7],'r', 'FaceAlpha',.5, 'EdgeColor','none'  );



% plot world
plot3(0,0,0,'.k', 'MarkerSize',40);grid on;hold on;
% plot anchors
plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'.m', 'MarkerSize',60);grid on;hold on;
plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'.y', 'MarkerSize',60);grid on;hold on;
% plot feet pos
plot3(w_pf1(1), w_pf1(2),w_pf1(3),'.b', 'MarkerSize',40);hold on;
plot3(w_pf2(1), w_pf2(2),w_pf2(3),'.b', 'MarkerSize',40);hold on;
% plot base pos
plot3(p_base(1), p_base(2),p_base(3),'.g', 'MarkerSize',40);hold on;

xlabel('$X$','interpreter', 'latex');
ylabel('$Y$','interpreter', 'latex');
zlabel('$Z$','interpreter', 'latex');

%plot rope forces
arrow3d_points(w_ph1,w_ph1+w_pr1 * fr1*force_scale,'color','m');grid on;hold on;
arrow3d_points(w_ph2,w_ph2+w_pr2 * fr2*force_scale,'color','y');grid on;hold on;
%plot feet forces 
%arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','b');grid on;hold on;
%arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','b');grid on;hold on;
 


%plot world reference frame
Tt = [eye(3), [0;0;0];
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
ht = triad('Parent',tt, 'linewidth', 6);


%plot base reference frame
Tt = [wRb, p_base;
    zeros(1,3) 1];
tt = hgtransform('Matrix', Tt);
ht = triad('Parent',tt, 'linewidth', 6);


%fundamental to see perpedicuolaity
xlim([0, 2])
ylim([min_y,max_y])
zlim([min_z,max_z])
axis equal
view(60,30)
pause(1)

    

 
