
close all 
clear all
clc

anchor_distance = 5;
p_anchor1 = [0;0;0];
p_anchor2 = [0;anchor_distance;0];
p_hoist_1 = [0;-0.05; 0.05];
p_hoist_2 = [0; 0.05; 0.05];

landing_joint = 0.7; % assumed always positive
lower_landing_leg = 0.3;
offset_base_y = 0.08;
p_foot_1 = [-lower_landing_leg*sin(landing_joint); -offset_base_y-lower_landing_leg*cos(landing_joint); 0.025]; % left foot
p_foot_2 = [-lower_landing_leg*sin(landing_joint); offset_base_y + lower_landing_leg*cos(landing_joint); 0.025]; % right foot

wall_normal = [1;0;0];

%p_base = [0.5;3.5; -6];
p_base = [0.16932 ; 3.95809; -4.19946];


orient_base = [0;0;0];

dt = 0.001;
time = [0:dt:10];

figure


vel_base = [0;0.5;0];
omega_base =[0;0;0];

%new model base frame (psi is associated to the rope plane so we have
%a rotation of (pi/2 -psi) about the y axis
wRb = eulToRot(orient_base);

z_axis_base = wRb(:,3);
y_axis_base = wRb(:,2);

% hoist positions in WF
w_ph1 = wRb*p_hoist_1 +p_base;
w_ph2 = wRb*p_hoist_2 +p_base; 


% feet positions in WF
w_pl1 = wRb*p_foot_1 +p_base;
w_pl2 = wRb*p_foot_2 +p_base; 

% line of actions of the anchor forces (univ vectors)
w_vr1  = (p_base-p_anchor1)/norm(p_base-p_anchor1);
w_vr2  = (p_base-p_anchor2)/norm(p_base-p_anchor2);

%correct way
% v_l1 = y_axis_base*y_axis_base'*(vel_base -cross_mx(w_pl1 - p_base)*omega_base)
% v_l2 = y_axis_base*y_axis_base'*(vel_base -cross_mx(w_pl2 - p_base)*omega_base)
% v_r1 = w_vr1'*(vel_base -cross_mx(w_ph1 - p_base)*omega_base)
% v_r2 = w_vr2'*(vel_base -cross_mx(w_ph2 - p_base)*omega_base)

%correct way with jacobian
Jmv = [y_axis_base'*eye(3) -y_axis_base'*cross_mx(w_pl1 - p_base),
       y_axis_base'*eye(3) -y_axis_base'*cross_mx(w_pl2 - p_base),
       w_vr1'*eye(3) -w_vr1'*cross_mx(w_ph1 - p_base),
       w_vr2'*eye(3) -w_vr2'*cross_mx(w_ph2 - p_base)];
y= Jmv*[vel_base; omega_base];

v_l1 = y_axis_base*y(1)
v_l2 = y_axis_base*y(2)
v_r1 = y(3)
v_r2 = y(4)
R = 0.15/2;
wheel_l = y(1)/R
wheel_r = y(2)/R

%already constraining v_l1 and v_l2 to be in Y (baselink) direction
% J =[ y_axis_base                         y_axis_base                                              w_vr1                                    w_vr2,
%      pinv(cross_mx(w_pl1 - p_base))*y_axis_base pinv(cross_mx(w_pl2 - p_base))*y_axis_base pinv(cross_mx(w_ph1 - p_base))*w_vr1     pinv(cross_mx(w_ph2 - p_base))*w_vr2];
% 
% W = diag([1 1 10 10 10 10 ]);
% wpsdinvJ = inv(J'*W*J + 1e-8*eye(4))*J'*W; %NB to have same results with pinv (svd decomp) we need to add the damping factor to be sure J'*J is invertible (eg J is not rank deficient)
% y= wpsdinvJ*[vel_base; omega_base];
% %y= pinv(J)*[vel_base; omega_base];
% 
% % check error 
% residuals = [vel_base; omega_base] - J*y 
% 
% v_l1 = y_axis_base*y(1)
% v_l2 = y_axis_base*y(2)
% v_r1 = y(3)
% v_r2 = y(4)


%not including constraints
% J =[ eye(3)                         eye(3)                         w_vr1                                    w_vr2,
%      pinv(cross_mx(w_pl1 - p_base)) pinv(cross_mx(w_pl2 - p_base)) pinv(cross_mx(w_ph1 - p_base))*w_vr1     pinv(cross_mx(w_ph2 - p_base))*w_vr2,
%      wall_normal'                   zeros(1,3)                       0                                         0,
%      zeros(1,3)                  wall_normal'                        0                                         0,
%      z_axis_base'                   zeros(1,3)                       0                                         0,
%      zeros(1,3)                   z_axis_base'                       0                                         0]
% 
% W = diag([1 1 1 1 1 1 25 25 10 10]);
% wpsdinvJ = inv(J'*W*J+1e-06*eye(8))*J'*W;
% y= wpsdinvJ*[vel_base; omega_base; zeros(4,1)];
% 
% % check error 
% residuals = [vel_base; omega_base; zeros(4,1)] - J*y 
% 
% v_l1 = y(1:3)
% v_l2 = y(4:6)
% v_r1 = y(7)
% v_r2 = y(8)




min_z = p_base(3)-2;
max_z = 0;
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
fill3([0.001,0.001,0.001,0.001] , [p_anchor1(2),p_anchor1(2) , p_anchor2(2),  p_anchor2(2)], [-7,0, 0, -7],'r', 'FaceAlpha',.5, 'EdgeColor','none'  );



% plot world
plot3(0,0,0,'.k', 'MarkerSize',40);grid on;hold on;
% plot anchors
plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'.m', 'MarkerSize',60);grid on;hold on;
plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'.y', 'MarkerSize',60);grid on;hold on;
% plot feet pos
plot3(w_pl1(1), w_pl1(2),w_pl1(3),'.b', 'MarkerSize',40);hold on;
plot3(w_pl2(1), w_pl2(2),w_pl2(3),'.b', 'MarkerSize',40);hold on;
% plot base pos
plot3(p_base(1), p_base(2),p_base(3),'.g', 'MarkerSize',40);hold on;
xlabel('$X$','interpreter', 'latex');
ylabel('$Y$','interpreter', 'latex');
zlabel('$Z$','interpreter', 'latex');

%plot rope vel
arrow3d_points(p_base,p_base+w_vr1 * v_r1*50,'color','r');grid on;hold on;
arrow3d_points(p_base,p_base+w_vr2 * v_r2*50,'color','r');grid on;hold on;

 arrow3d_points(w_pl1,w_pl1+v_l1*10 ,'color','b');grid on;hold on;
arrow3d_points(w_pl2,w_pl2+v_l2*10,'color','b');grid on;hold on;



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
% ylim([min_y,max_y])
zlim([min_z,max_z])
axis equal
view(60,30)
grid on 



