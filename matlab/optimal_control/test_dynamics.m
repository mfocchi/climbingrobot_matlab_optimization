state0 = [ 0.626            0            3          0.5          0.1            0];
K = 10;
N_dyn = 40
Tf = 1.5
dt_dyn = Tf / N_dyn;
dt = 0.001;

figure

[states, t] = integrate_dynamics(state0, dt, floor(Tf/dt), K);
[states_rough, t_rough] = integrate_dynamics(state0,dt_dyn, N_dyn, K);
plot(t_rough, states_rough(1,:),'-bo'); hold on
plot(t, states(1,:),'r'); 

