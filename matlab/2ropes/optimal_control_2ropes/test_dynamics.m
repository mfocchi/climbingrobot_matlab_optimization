close all 
clear all 

global m g b  Fr_r Fr_l   p_a1 p_a2 T_th



%WORLD FRAME ATTACHED TO ANCHOR 1
anchor_distance = 5;
b = anchor_distance;
T_th = 0.05;
p_a1 = [0;0;0];
p_a2 = [0;anchor_distance;0];
g = 9.81;
m = 5.08;   % Mass [kg]

%jump params
p0 = [0.005; 2.5; -6]; % there is singularity for px = 0!


%compute initial state from jump param
x0 =  computeStateFromCartesian(p0);

m = 5.08; 
g = 9.81;
anchor_distance = 5;
b = anchor_distance;

Tf =  1.941;
dt = 0.01;
%inputs
n_sim_steps = floor(Tf/dt);
Fleg = [200;0;0];
Fr_l = ones(n_sim_steps) * -40;
Fr_r = ones(n_sim_steps) * -30;

[~,~,x, t] = integrate_dynamics(x0, 0, dt, floor(Tf/dt), Fr_l, Fr_r, Fleg,'euler');

for i=1:length(x)    
    [X(i), Y(i), Z(i)] = forwardKin(x(1,i), x(2,i), x(3,i));
end

figure
subplot(3,1,1)
plot(t, X ,'-ro'); 
grid on;
subplot(3,1,2)
plot(t, Y ,'-ro');
grid on;
subplot(3,1,3)
plot(t, Z ,'-ro');
grid on;
