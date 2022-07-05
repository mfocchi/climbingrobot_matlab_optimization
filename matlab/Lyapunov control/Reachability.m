clear all;
clc;

syms theta phi l omega psi r m Fr Fn Ft g real;

q = [theta;
    phi;
    l;
    omega;
    psi;
    r];

f = [omega;
psi;
r;
- 2*omega*r/l + psi^2*sin(theta)*cos(theta) - g/l*sin(theta);
- 2*psi*r/l - 2*psi*omega*cos(theta)/sin(theta);
l*(omega^2 + psi^2*sin(theta)^2) + g*cos(theta)];

g_omega = [0; 0; 0; 1/(m*l); 0; 0];
g_psi = [0; 0; 0; 0; 1/(m*l*sin(theta)); 0];
g_r = [0; 0; 0; 0; 0; 1/m];

%% Fully controllable

Delta0 = [g_omega, g_psi, g_r];

% Lie-brackets
Lie_f_g_omega = simplify(jacobian(g_omega, q)*f - jacobian(f, q)*g_omega);
Lie_f_g_psi = simplify(jacobian(g_psi, q)*f - jacobian(f, q)*g_psi);
Lie_f_g_r = simplify(jacobian(g_r, q)*f - jacobian(f, q)*g_r);

Delta1 = [Delta0, Lie_f_g_omega, Lie_f_g_psi, Lie_f_g_r]

rank(Delta1)


%% With just the rope

Delta0 = g_r;

% Lie-brackets
Lie_f_g_r = simplify(jacobian(g_r, q)*f - jacobian(f, q)*g_r);

Delta1 = [Delta0, Lie_f_g_r]

rank(Delta1)

% Lie-Brackets
Lie_f2_g_r = simplify(jacobian(Lie_f_g_r, q)*f - jacobian(f, q)*Lie_f_g_r);

Delta2 = [Delta1, Lie_f2_g_r]

rank(Delta2)

% Lie-Brackets
Lie_f3_g_r = simplify(jacobian(Lie_f2_g_r, q)*f - jacobian(f, q)*Lie_f2_g_r);

Delta3 = [Delta2, Lie_f3_g_r]

rank(Delta3)

% Lie-Brackets
Lie_f4_g_r = simplify(jacobian(Lie_f3_g_r, q)*f - jacobian(f, q)*Lie_f3_g_r);

Delta4 = [Delta3, Lie_f4_g_r]

rank(Delta4)

% Lie-Brackets
Lie_f5_g_r = simplify(jacobian(Lie_f4_g_r, q)*f - jacobian(f, q)*Lie_f4_g_r);

Delta5 = [Delta4, Lie_f5_g_r]

rank(Delta5)
