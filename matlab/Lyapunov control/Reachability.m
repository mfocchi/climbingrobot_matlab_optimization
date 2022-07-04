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

Delta0 = [g_omega, g_psi, g_r]

% Lie-brackets
Lie_f_g_omega = simplify(jacobian(g_omega, q)*f - jacobian(f, q)*g_omega)
Lie_f_g_psi = simplify(jacobian(g_psi, q)*f - jacobian(f, q)*g_psi)
Lie_f_g_r = simplify(jacobian(g_r, q)*f - jacobian(f, q)*g_r)

Delta1 = [Delta0, Lie_f_g_omega, Lie_f_g_psi, Lie_f_g_r]



