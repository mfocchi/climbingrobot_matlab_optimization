
%https://en.wikipedia.org/wiki/Dirac_delta_function

% additional data for plotting
T_th   = 0.05; %on zeta variable
sigma_gauss = T_th/6;
mu_gauss    = T_th/2;

syms x
f = 1/sqrt(2*pi*sigma_gauss^2)*exp(-(x-mu_gauss)^2/(2*sigma_gauss^2))
fplot(f, [-1, 1])
vpa(int(f, [-1,1]))