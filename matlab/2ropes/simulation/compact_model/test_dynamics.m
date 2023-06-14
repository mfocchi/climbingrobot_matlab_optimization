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


int_method = 'rk4';
%compute initial state from jump param
x0 =  computeStateFromCartesian(p0);
dt = 0.001;
Tf =  1.9418;
%inputs
n_sim_steps = floor(Tf/dt);
Fleg = [200;0;0];
Fr_l = ones(n_sim_steps) * -40;
Fr_r = ones(n_sim_steps) * -30;

[~,~,x, t] = integrate_dynamics(x0, 0, dt, floor(Tf/dt), Fr_l, Fr_r, Fleg,int_method);

for i=1:length(x)    
    [X(i), Y(i), Z(i)] = forwardKin(x(1,i), x(2,i), x(3,i));
end

final_point = [X(end),Y(end),Z(end)];
ode45_final_point =[ -0.0000,    1.6240,   -1.6246];
fprintf('Compare with ode45 final point [%3.4f, %3.4f, %3.4f] \n', final_point- ode45_final_point)



% figure
% subplot(3,1,1)
% plot(t, X ,'-ro'); 
% grid on;
% subplot(3,1,2)
% plot(t, Y ,'-ro');
% grid on;
% subplot(3,1,3)
% plot(t, Z ,'-ro');


% check with substep integration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N_dyn = 50;
dt_dyn = Tf / N_dyn;
int_steps = 10;
Fr_l_rough = ones(N_dyn) * -40;
Fr_r_rough = ones(N_dyn) * -30;
[states_rough_sub, t_rough_sub] = computeRollout(x0, 0,dt_dyn, N_dyn, Fr_l_rough, Fr_r_rough,Fleg,int_method, int_steps);

% check without substep integrtion (don not give int_steps as input)
[states_rough, t_rough] = computeRollout(x0, 0,dt_dyn, N_dyn, Fr_l_rough, Fr_r_rough,Fleg,int_method);



figure

subplot(3,1,1)
plot(t_rough, states_rough(1,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(1,:),'ko'); hold on
plot(t, x(1,:),'-r'); 
legend('rough','rough sub','cont');
ylabel('psi')

subplot(3,1,2)
plot(t_rough, states_rough(2,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(2,:),'ko'); hold on
plot(t, x(2,:),'-r');
ylabel('l1')

subplot(3,1,3)
plot(t_rough, states_rough(3,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(3,:),'ko'); hold on
plot(t, x(3,:),'-r');
ylabel('l2')


