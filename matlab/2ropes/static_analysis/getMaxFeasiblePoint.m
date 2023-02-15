

function out = getMaxFeasiblePoint(baseline, mu, wall_clearance, rope_length)


p_anchor1 = [0;-5;0];
p_anchor2 = [0;5;0];


g_vec = [0;0;-9.81];
mass = 7;
min_feet_force = 0;

for step=3:0.1:7
    % base pos
    p_base = [wall_clearance; 0+step; -rope_length];

    %old model base frame orientation
    % [theta, phi, l] = computePolarVariables(p_base);
    % wRb =   [cos(phi)*sin(theta), -sin(phi), cos(phi)*cos(theta) 
    %          sin(phi)*sin(theta),  cos(phi), cos(theta)*sin(phi) 
    %          -cos(theta),         0,          sin(theta)        ]

    [psi] = computeTheta(p_base);
    %new model base frame (psi is associated to the rope plane so we have
    %a rotation of (pi/2 -psi) about the y axis
    wRb =[ cos(pi/2-psi), 0, sin(pi/2-psi),
               0, 1,          0, 
            -sin(pi/2-psi), 0, cos(pi/2-psi)]


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
    Aeq2 = [cross_mx(w_pf1 - p_base) cross_mx(w_pf2 - p_base) zeros(3,1) zeros(3,1)] ;
    beq2 = zeros(3,1);

    % stack equality constgraints
    Aeq = [Aeq1;Aeq2];
    beq = [beq1;beq2];

    %friction cone matrix
    %contact frame
    t1 = [0;0;1];
    t2 = [0;1;0];
    n = [1;0;0];
    %friction cone pyramid
    F = [(t1-mu*n)';
         (-t1-mu*n)';
         (t2-mu*n)';
         (-t2-mu*n)'];

    % inequalities friction cones constraints and rope unilaterality of rope
    % forces
    C = blkdiag([F ;-n'],[F; -n'], -1,-1);
    d = [zeros(4,1);-min_feet_force; zeros(4,1);-min_feet_force;0;0];

    % only unilateral on feet and rope used for optim
    %C = blkdiag(-n',-n', -1,-1);
    %d = [-min_feet_force; -min_feet_force;0;0]

    [x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, C, d, Aeq, beq);
    %[x,FVAL,EXITFLAG] = quadprog_solve_qp(G, g, [], [], Aeq, beq);
    fl1 = x(1:3);
    fl2 = x(4:6);
    fr1 = x(7);
    fr2 = x(8);

    if EXITFLAG == -2
        break;
        
    end

%     margins = C*x - d;
%     S_n1 = [zeros(1,4) 1 zeros(1,5) 0 0] ;
%     S_n2 = [zeros(1,5) zeros(1,4) 1 0 0] ;
%     S_F1 = blkdiag(eye(4), zeros(1+5+1+1,1+5+1+1 ));
%     S_F2 = blkdiag(zeros(5,5), eye(4),zeros(1+1+1,1+1+1) );
% 
%     normal1_viol = S_n1*margins > 0;
%     normal2_viol = S_n2*margins > 0;
%     F1_viol = any(S_F1*margins  > 0);
%     F2_viol = any(S_F2*margins > 0);
%     if normal1_viol 
%         disp('normal 1 violation')
%     end
%     if normal2_viol 
%         disp('normal 2 violation')
%     end
%     if F1_viol 
%         disp('friction 1 violation')
%     end
%     if F2_viol 
%         disp('friction 2 violation')
%     end
        
    out = p_base;   

    
end


