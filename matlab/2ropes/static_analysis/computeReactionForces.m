
close all 
clear all
clc

p_anchor1 = [0;-5;0]
p_anchor2 = [0;5;0]

% x = [fl1, fl2, fr1, fr2]
baseline = 0.4;
wall_clearance = 0.9;

mu = 0.8
g_vec = [0;0;-9.81]
mass = 5
force_scale =0.1
min_feet_force = 0;

figure

for step=3:0.1:6
    % base pos
    p_base = [wall_clearance; 0+step; -5];

    %old model base frame orientation
    % [theta, phi, l] = computePolarVariables(p_base);
    % wRb =   [cos(phi)*sin(theta), -sin(phi), cos(phi)*cos(theta) 
    %          sin(phi)*sin(theta),  cos(phi), cos(theta)*sin(phi) 
    %          -cos(theta),         0,          sin(theta)        ]

    [theta] = computeTheta(p_base);
    %new model base frame
    wRb =[ sin(theta), 0, cos(theta),
               0, 1,          0, 
            -cos(theta), 0, sin(theta)]


    pf1 = [0; -baseline/2; -wall_clearance];
    pf2 = [0; baseline/2; -wall_clearance];

    w_pf1 = wRb*pf1 +p_base;
    w_pf2 = wRb*pf2 +p_base; 

    % line of actions of the anchor forces (univ vectors)
    w_pr1  = (p_anchor1 - p_base)/norm(p_anchor1 - p_base);
    w_pr2  = (p_anchor2 - p_base)/norm(p_anchor2 - p_base);



    % optim var are fl1(3x1), fl2(3x1), fr1 (scalar), fr2(scalar)

    % #∥Px−p∥2# 
    % # G =  Mt*M
    % # g = -MT*b
    % #0.5 xT*G*x + g*x
    % #s.t. Cx≤d
    % #     Ax=b

    w_feet  = 0.1
    w_rope = 0.01
    M = blkdiag(w_feet*eye(3),w_feet*eye(3),w_rope,w_rope);
    m = zeros(3+3+1+1,1);
    G = 2*M'* M;
    g = -2*m'* M;

    % C = [1., 2., 1.; 2., 0., 1.; -1., 2., -1.];
    % d = [3.; 2.; -2.];

    % force equilibrium  constraints (3)
    Aeq1 = [eye(3) eye(3) w_pr1 w_pr2] 
    beq1 = -mass*g_vec;

    % moment equilibrium  constraints about base origin(3)
    Aeq2 = [cross_mx(w_pf1 - p_base) cross_mx(w_pf2 - p_base) zeros(3,1) zeros(3,1)] 
    beq2 = zeros(3,1);

    % stack equality constgraints
    Aeq = [Aeq1;Aeq2];
    beq = [beq1;beq2];

    %friction cone matrix
    %contact frame
    t1 = [0;0;1]
    t2 = [0;1;0]
    n = [1;0;0]
    %friction cone pyramid
    F = [(t1-mu*n)';
         (-t1-mu*n)';
         (t2-mu*n)';
         (-t2-mu*n)']

    % inequalities friction cones constraints and rope unilaterality of rope
    % forces
    C = blkdiag([F ;-n'],[F; -n'], -1,-1);
    d = [zeros(4,1);-min_feet_force; zeros(4,1);-min_feet_force;0;0]

    % only unilateral on feet and rope used for optim
    %C = blkdiag(-n',-n', -1,-1);
    %d = [-min_feet_force; -min_feet_force;0;0]

    [x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, C, d, Aeq, beq);
    %[x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, [], [], Aeq, beq);
    fl1 = x(1:3)
    fl2 = x(4:6)
    fr1 = x(7)
    fr2 = x(8)

    if EXITFLAG == -2

        disp('no solution found')
    end

    margins = C*x - d;
    S_n1 = [zeros(1,4) 1 zeros(1,5) 0 0] ;
    S_n2 = [zeros(1,5) zeros(1,4) 1 0 0] ;
    S_F1 = blkdiag(eye(4), zeros(1+5+1+1,1+5+1+1 ));
    S_F2 = blkdiag(zeros(5,5), eye(4),zeros(1+1+1,1+1+1) );

    normal1_viol = S_n1*margins > 0;
    normal2_viol = S_n2*margins > 0;
    F1_viol = any(S_F1*margins  > 0);
    F2_viol = any(S_F2*margins > 0);
    if normal1_viol 
        disp('normal 1 violation')
    end
    if normal2_viol 
        disp('normal 2 violation')
    end
    if F1_viol 
        disp('friction 1 violation')
    end
    if F2_viol 
        disp('friction 2 violation')
    end


    %plot
    % fl1 = [1; 0;0];
    % fl2 = [1; 0;0];
    % fr1 = 1;
    % fr2 = 1;

    min_z = -7;
    max_z = 1;
    min_y = -12;
    max_y = 12;
    
    
    clf(gcf)


    % plot world
    plot3(0,0,0,'.k', 'MarkerSize',40);grid on;hold on;
    % plot anchors
    plot3(p_anchor1(1),p_anchor1(2),p_anchor1(3),'.m', 'MarkerSize',40);grid on;hold on;
    plot3(p_anchor2(1),p_anchor2(2),p_anchor2(3),'.y', 'MarkerSize',40);grid on;hold on;
    % plot feet pos
    plot3(w_pf1(1), w_pf1(2),w_pf1(3),'.b', 'MarkerSize',40);hold on;
    plot3(w_pf2(1), w_pf2(2),w_pf2(3),'.b', 'MarkerSize',40);hold on;
    % plot base pos
    plot3(p_base(1), p_base(2),p_base(3),'.g', 'MarkerSize',40);hold on;

    %plot rope forces
    arrow3d_points(p_base,p_base+w_pr1 * fr1*force_scale,'color','m');grid on;hold on;
    arrow3d_points(p_base,p_base+w_pr2 * fr2*force_scale,'color','y');grid on;hold on;
    %plot feet forces
    if normal1_viol || F1_viol
        arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','k');grid on;hold on;
    else
        arrow3d_points(w_pf1,w_pf1+fl1*10*force_scale,'color','b');grid on;hold on;
    end

    if normal2_viol || F2_viol
       arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','k');grid on;hold on;

    else
       arrow3d_points(w_pf2,w_pf2+fl2*10*force_scale,'color','b');grid on;hold on;
    end

    %     drawing a wall at X = 0
    p1 = [0 min_y min_z];
    p2 = [0 max_y min_z];
    p3 = [0 max_y max_z];
    p4 = [0 min_y max_z];
    Xw = [p1(1) p2(1) p3(1) p4(1)];
    Yw = [p1(2) p2(2) p3(2) p4(2)];
    Zw = [p1(3) p2(3) p3(3) p4(3)];
    h = fill3(Xw, Yw, Zw, 'b', 'FaceAlpha',.5  );
    %plot world reference frame
    Tt = [eye(3), [0;0;0];
        zeros(1,3) 1];
    tt = hgtransform('Matrix', Tt);
    ht = triad('Parent',tt, 'linewidth', 6);


    %plot base reference frame
    % Tt = [wRb, p_base;
    %     zeros(1,3) 1];
    % tt = hgtransform('Matrix', Tt);
    % ht = triad('Parent',tt, 'linewidth', 6);


    %fundamental to see perpedicuolaity
    xlim([0, 2])
    ylim([min_y,max_y])
    zlim([min_z,max_z])
    axis equal
    view(60,30)
    pause(1)

    
end


