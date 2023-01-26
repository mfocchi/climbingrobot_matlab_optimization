close all 
clear all 

global m g

state0 = [ 0.626            0            3          0.5          0.1            0];


m = 5;
g = 9.81;


N_dyn = 20
Tf = 1.5
dt_dyn = Tf / N_dyn;
dt = 0.001;
int_steps = 10;

Fr = linspace(1,50,floor(Tf/dt));
Fr_rough = linspace(1,50, N_dyn);

for i=1:N_dyn              
    if (i>=2)     
      [states_rough_sub(:,i), t_rough_sub(i)] = integrate_dynamics(states_rough_sub(:,i-1), t_rough_sub(i-1), dt_dyn/(int_steps-1), int_steps, Fr_rough(i-1)*ones(1,int_steps), 'rk4'); % keep Fr constant           
    else
      states_rough_sub(:,i) = state0;
      t_rough_sub(i) = 0;      
    end    
end

[~,~,states, t] = integrate_dynamics(state0,0, dt, floor(Tf/dt), Fr,'rk4');
[~,~,states_rough, t_rough] = integrate_dynamics(state0, 0,dt_dyn, N_dyn, Fr,'rk4');

figure
plot(t_rough, states_rough(1,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(1,:),'ko'); hold on
plot(t, states(1,:),'-r'); 
legend('rough','rough sub','cont');

figure
plot(t_rough, states_rough(2,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(2,:),'ko'); hold on
plot(t, states(2,:),'-r');
legend('rough','rough sub','cont');

figure
plot(t_rough, states_rough(3,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(3,:),'ko'); hold on
plot(t, states(3,:),'-r');
legend('rough','rough sub','cont');
